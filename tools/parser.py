"""Parse the current acquisition format and persistent diagnostic packets.

Usage: python parser.py data_0011.bin --out parsed_0011
See README.md for timestamp assumptions and units. Standard library only.
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
from lsm_timing import TimingEstimator
from diag_format import DETAIL_SIZES, DIAG_NAMES, SENSOR_FIELDS, GLOBAL_FIELDS, decode_health, decode_detail
import struct
import wave
from collections import Counter
from contextlib import ExitStack
from pathlib import Path

HEADER = struct.Struct('<BBII')
MODULO = 1 << 32
SIZES = {5: 12, 6: 12, 7: 512, 8: 0, 255: 0, **DETAIL_SIZES}
NAMES = {5: 'bme280_main', 6: 'bme280_sat', 7: 'microphone',
         17: 'adxl371_main', 18: 'adxl371_sat',
         19: 'lsm_main', 20: 'lsm_sat', **DIAG_NAMES}
ADXL_COLUMNS = ['timestamp_us', 'raw_x', 'raw_y', 'raw_z', 'g_x', 'g_y', 'g_z']
BME_COLUMNS = ['timestamp_us', 'temperature_c', 'pressure_pa', 'humidity_rh']
LSM_M_S2_PER_LSB = 0.000976 * 9.80665  # ±32 g; original 0.00976 was ~2% high.


class ParseError(ValueError):
    pass


class ClockUnwrapper:
    """Per-sensor monotonic clocks, with first observations aligned across wrap.

    Consecutive packets for one sensor must be less than 2**31 us apart.
    Small backward timestamps are errors, not silently treated as rollovers.
    """
    def __init__(self):
        self.previous = {}
        self.latest = None

    def unwrap(self, sensor, raw):
        if sensor in self.previous:
            prev_raw, prev_time = self.previous[sensor]
            delta = (raw - prev_raw) % MODULO
            if delta >= MODULO // 2:
                raise ParseError(f'Sensor {sensor}: backward/ambiguous timestamp {prev_raw} -> {raw}')
            result = prev_time + delta
        elif self.latest is None:
            result = raw
        else:
            epoch = (self.latest - raw + MODULO // 2) // MODULO
            result = raw + epoch * MODULO
        self.previous[sensor] = raw, result
        self.latest = result if self.latest is None else max(self.latest, result)
        return result


def sample_timestamp(packet_timestamp, index, count, rate):
    # For this firmware, watermark == packet size; oldest batch ends at IRQ.
    # ISR latency, ODR error, filter delay and overrun cannot be recovered here.
    return packet_timestamp - (count - 1 - index) * 1e6 / rate


def iter_packets(stream, resync=False, stats=None):
    if stats is None:
        stats = Counter()
    while True:
        offset = stream.tell()
        raw = stream.read(HEADER.size)
        if not raw:
            return
        if len(raw) != HEADER.size:
            raise ParseError(f'Truncated header at byte {offset}')
        sync, sensor, timestamp, size = HEADER.unpack(raw)
        if sync != 0xAA or not valid_payload_length(sensor, size):
            if not resync:
                raise ParseError(f'Invalid header at byte {offset}: sync={sync:#x}, id={sensor}, length={size}')
            # Advance one byte. The original seek(-10) retried the same header forever.
            stream.seek(offset + 1)
            stats['resync_skipped_bytes'] += 1
            continue
        payload = stream.read(size)
        if len(payload) != size:
            raise ParseError(f'Truncated sensor {sensor} payload at byte {offset}')
        yield offset, sensor, timestamp, payload
        if sensor in (8, 255):
            return


def csv_writer(stack, path, columns):
    handle = stack.enter_context(path.open('w', newline='', encoding='utf-8'))
    writer = csv.writer(handle)
    writer.writerow(columns)
    return writer


ACQUISITION_META = struct.Struct('<IIIIIHHHh')
HIGH_RATE_IDS = (17, 18, 19, 20)


def valid_payload_length(sensor, length):
    if sensor in HIGH_RATE_IDS:
        width, limit = (6, 170) if sensor <= 18 else (7, 300)
        return 28 <= length <= 28 + limit * width and (length - 28) % width == 0
    return sensor in SIZES and length == SIZES[sensor]


def scan_lsm_timing(source, reference_rate, resync=False):
    estimator = TimingEstimator()
    clock, framing = ClockUnwrapper(), Counter()
    try:
        with source.open('rb') as stream:
            for offset, sensor, raw_time, payload in iter_packets(stream, resync, framing):
                time = clock.unwrap(sensor, raw_time)
                if sensor not in (19, 20):
                    continue
                seq, epoch, irq, acc, gyro, entries, count, flags, temp = ACQUISITION_META.unpack(payload[:28])
                if len(payload) != 28+7*count or count > 300:
                    raise ParseError(f'Acquisition count/length mismatch at byte {offset}')
                if count and not flags & 0xde:
                    estimator.add(sensor, epoch, time, acc, gyro, entries, count)
        if framing:
            return {}, [], 'automatic fits disabled after framing recovery'
    except ParseError as exc:
        return {}, [], 'automatic fits disabled: '+str(exc)
    models, report = estimator.finish(reference_rate)
    return models, report, None


class AcquisitionDecoder:
    def __init__(self, stack, output, adxl_rate, lsm_rate, fits=None, rate_overrides=None, emit_rows=True):
        self.adxl_rate, self.lsm_rate = adxl_rate, lsm_rate
        # Native decoder writes the high-volume rows; Python retains metadata audit.
        self.emit_rows = emit_rows
        self.fits = fits or {}
        self.rate_overrides = rate_overrides or {}
        self.previous_sequence = {}
        self.epochs = {}
        self.summary = {'records': {}, 'missing_batch_sequences': {}, 'events': 0,
                        'timestamp_model': 'ADXL IRQ/service anchor; LSM selected per-epoch timing model',
                        'lsm_timing_models': [], 'lsm_index_gaps': {}, 'lsm_tag_step_anomalies': {}}
        self.batch_writer = csv_writer(stack, output / 'acquisition_batches.csv',
            ['sensor_id', 'sequence', 'epoch', 'file_offset', 'read_start_us', 'irq_us',
             'fifo_entries_before', 'record_count', 'flags_hex', 'first_acc_index', 'first_gyro_index'])
        self.adxl = {}
        self.lsm = {}
        extra = ['sequence', 'epoch', 'sample_index', 'read_start_us', 'irq_us', 'flags_hex', 'timing_model']
        for sid in (() if not emit_rows else (17, 18)):
            self.adxl[sid] = csv_writer(stack, output / (NAMES[sid] + '.csv'), ADXL_COLUMNS + extra)
        for sid, loc in ([] if not emit_rows else [(19, 'main'), (20, 'sat')]):
            for kind in ('accel', 'gyro'):
                values = ['acc_x', 'acc_y', 'acc_z'] if kind == 'accel' else ['gyr_x', 'gyr_y', 'gyr_z']
                columns = ['timestamp_us', 'raw_x', 'raw_y', 'raw_z'] + values + ['temperature_c', 'tag_byte', 'tag_counter'] + extra
                self.lsm[sid, kind] = csv_writer(stack, output / f'lsm_{loc}_{kind}.csv', columns)

    def decode(self, sensor, service_time, payload, offset):
        seq, epoch, irq_raw, acc_first, gyro_first, entries, count, flags, temp = ACQUISITION_META.unpack(payload[:28])
        width, limit = (6, 170) if sensor <= 18 else (7, 300)
        if count > limit or len(payload) != 28 + width * count:
            raise ParseError(f'Acquisition count/length mismatch at byte {offset}')
        previous = self.previous_sequence.get(sensor)
        if previous is not None:
            delta = (seq - previous) % MODULO
            if delta == 0 or delta >= MODULO // 2:
                raise ParseError(f'Acquisition sequence reversed/duplicated at byte {offset}')
            if delta > 1:
                name = NAMES[sensor]
                self.summary['missing_batch_sequences'][name] = self.summary['missing_batch_sequences'].get(name, 0) + delta - 1
        self.previous_sequence[sensor] = seq
        irq = ''
        if flags & 1:
            delta = (irq_raw - service_time % MODULO + MODULO // 2) % MODULO - MODULO // 2
            irq = service_time + delta
        self.batch_writer.writerow([sensor, seq, epoch, offset, service_time, irq, entries, count, hex(flags), acc_first, gyro_first])
        # Reset/error events carry no data; never stitch their surrounding epochs.
        if flags & 0xde:
            self.summary['events'] += 1
            if count:
                raise ParseError(f'Acquisition fault event unexpectedly carries data at byte {offset}')
        if not count:
            return
        name = NAMES[sensor]
        self.summary['records'][name] = self.summary['records'].get(name, 0) + count
        if sensor <= 18:
            if not self.emit_rows:
                return
            raw = struct.unpack(f'<{count*3}h', payload[28:])
            dt = 1e6 / self.adxl_rate
            first_time = irq - 99 * dt if irq != '' else service_time - (count - 1) * dt
            model = 'watermark_irq_nominal' if irq != '' else 'read_start_nominal'
            for i in range(count):
                xyz = raw[i*3:i*3+3]
                self.adxl[sensor].writerow([first_time+i*dt, *xyz, *(v*.1 for v in xyz),
                    seq, epoch, i, service_time, irq, hex(flags), model])
            return
        key = sensor, epoch
        if key not in self.epochs:
            rate = self.rate_overrides.get(sensor, self.lsm_rate)
            dt = 1e6/rate
            fitted = self.fits.get(key) if sensor not in self.rate_overrides else None
            first_time = irq-149*dt if irq != '' else service_time-(max(1,(entries+1)//2)-1)*dt
            self.epochs[key] = {
                'accel_base': fitted['accel']['base_us'] if fitted else first_time-acc_first*dt,
                'gyro_base': fitted['gyro']['base_us'] if fitted else first_time-gyro_first*dt,
                'accel_rate': fitted['accel']['rate_hz'] if fitted else rate,
                'gyro_rate': fitted['gyro']['rate_hz'] if fitted else rate,
                'accel_next': acc_first, 'gyro_next': gyro_first,
                'accel_tag': None, 'gyro_tag': None,
                'model': 'epoch_fifo_mcu_affine_fit' if fitted else
                         ('epoch_user_rate_' if sensor in self.rate_overrides else 'epoch_nominal_')+
                         ('irq_anchor' if irq != '' else 'read_start_anchor')
            }
            e = self.epochs[key]
            self.summary['lsm_timing_models'].append({'sensor_id':sensor, 'epoch':epoch,
                'model':e['model'], 'accel_rate_hz':e['accel_rate'], 'gyro_rate_hz':e['gyro_rate'],
                'accel_base_us':e['accel_base'], 'gyro_base_us':e['gyro_base']})
        state = self.epochs[key]
        if not self.emit_rows:
            return  # C decoder checks per-sample index and tag continuity.
        if acc_first < state['accel_next'] or gyro_first < state['gyro_next']:
            raise ParseError(f'LSM sample index went backward inside epoch {epoch}')
        indices = {'accel': acc_first, 'gyro': gyro_first}
        for kind, index in indices.items():
            missing = index-state[kind+'_next']
            if missing:
                name = f'{sensor}_{kind}'
                self.summary['lsm_index_gaps'][name] = self.summary['lsm_index_gaps'].get(name,0)+missing
                state[kind+'_tag'] = None

        for i in range(count):
            tag, x, y, z = struct.unpack_from('<Bhhh', payload, 28+i*7)
            kind_id = (tag >> 3) & 31
            if kind_id not in (1, 2):
                raise ParseError(f'Unknown LSM tag {kind_id} at byte {offset}')
            kind = 'gyro' if kind_id == 1 else 'accel'
            index = indices[kind]
            indices[kind] += 1
            tag_count = (tag >> 1) & 3
            previous_tag = state[kind+'_tag']
            if previous_tag is not None and (tag_count-previous_tag) % 4 != 1:
                name = f'{sensor}_{kind}'
                self.summary['lsm_tag_step_anomalies'][name] = self.summary['lsm_tag_step_anomalies'].get(name,0)+1
            state[kind+'_tag'] = tag_count
            t = state[kind+'_base'] + index*1e6/state[kind+'_rate']
            scale = .07 if kind == 'gyro' else LSM_M_S2_PER_LSB
            temperature = '' if flags & 0x100 else 25+temp/256
            self.lsm[sensor, kind].writerow([t, x, y, z, x*scale, y*scale, z*scale,
                temperature, tag, (tag >> 1) & 3, seq, epoch, index, service_time, irq,
                hex(flags), state['model']])
        for kind, index in indices.items():
            state[kind+'_next'] = index


def _native_executable():
    """Only use a binary explicitly built alongside the parser source."""
    path = Path(__file__).with_name('fast_decode.exe' if os.name == 'nt' else 'fast_decode')
    return path if path.is_file() else None


def _run_native(source, output, fits, adxl_rate, lsm_rate, audio_rate,
                lsm_rate_main, lsm_rate_sat, resync, executable, summary_only=False):
    # The pre-scan provides accepted LSM epoch fits; C handles sample expansion.
    fit_path = output / '.native_fits.tsv'
    with fit_path.open('w', encoding='ascii') as f:
        for (sid, epoch), values in fits.items():
            a, g = values['accel'], values['gyro']
            f.write(f"{sid} {epoch} {a['rate_hz']:.17g} {a['base_us']:.17g} "
                    f"{g['rate_hz']:.17g} {g['base_us']:.17g}\n")
    command = [str(executable), '--input', str(source), '--out', str(output),
               '--fits', str(fit_path), '--adxl-rate', str(adxl_rate),
               '--lsm-rate', str(lsm_rate), '--audio-rate', str(audio_rate)]
    if lsm_rate_main is not None:
        command += ['--lsm-rate-main', str(lsm_rate_main)]
    if lsm_rate_sat is not None:
        command += ['--lsm-rate-sat', str(lsm_rate_sat)]
    if resync:
        command.append('--resync')
    if summary_only:
        command.append('--summary-only')
    try:
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise ParseError(f"Native decoder failed ({result.returncode}): {result.stderr.strip()}")
    finally:
        fit_path.unlink(missing_ok=True)


def _merge_native_integrity(output, acquisition_summary):
    path = output / '.native_integrity.tsv'
    if not path.is_file():
        raise ParseError('Native decoder did not produce an integrity report')
    continuity = {}
    for line in path.read_text(encoding='ascii').splitlines():
        parts = line.split('\t')
        kind, name = parts[:2]
        if kind == 'CONT':
            continuity[name] = dict(zip(('records', 'index_discontinuities',
                                         'unexpected_mod4_tag_steps'),
                                        (int(n) for n in parts[2:])))
        else:
            n = int(parts[2])
            if n:
                field = 'lsm_index_gaps' if kind == 'GAP' else 'lsm_tag_step_anomalies'
                acquisition_summary[field][name] = n
    acquisition_summary['lsm_continuity'] = continuity
    path.unlink()


def parse_binary(input_file, output_dir=None, adxl_rate=5120.0, lsm_rate=6670.0,
                 audio_rate=44100, resync=False, lsm_timing="auto",
                 lsm_rate_main=None, lsm_rate_sat=None, backend="auto",
                 summary_only=False):
    rates = [adxl_rate, lsm_rate, audio_rate]+[r for r in (lsm_rate_main,lsm_rate_sat) if r is not None]
    if any(not math.isfinite(r) or r <= 0 for r in rates):
        raise ValueError('Sample rates must be positive')
    if backend not in ('auto', 'python', 'native'):
        raise ValueError('backend must be auto, python or native')
    executable = _native_executable() if backend != 'python' else None
    if (backend == 'native' or summary_only) and executable is None:
        raise ValueError('Native binary required. Run: python build_fast_parser.py')
    if summary_only and backend == 'python':
        raise ValueError('--summary-only requires the native C decoder')
    use_native = executable is not None
    if lsm_timing not in ('auto','nominal'):
        raise ValueError('lsm_timing must be auto or nominal')
    source = Path(input_file)
    output = Path(output_dir) if output_dir else source.with_name(source.stem + '_parsed')
    # Fail before creating outputs if the input does not exist.
    with source.open('rb'):
        pass
    fits, fit_report, fit_warning = {}, [], None
    if lsm_timing == 'auto':
        fits, fit_report, fit_warning = scan_lsm_timing(source, lsm_rate, resync)
    overrides = {sid:r for sid,r in [(19,lsm_rate_main),(20,lsm_rate_sat)] if r is not None}
    output.mkdir(parents=True, exist_ok=False)
    if use_native:
        _run_native(source, output, fits, adxl_rate, lsm_rate, audio_rate,
                    lsm_rate_main, lsm_rate_sat, resync, executable, summary_only)
    counts, framing = Counter(), Counter()
    clock = ClockUnwrapper()
    first_irq, last_irq = {}, {}
    audio_frame = 0
    termination = None
    failure = None
    health_snapshots = []
    detail_counts = Counter()
    last_health = None
    with ExitStack() as stack:
        acquisition = AcquisitionDecoder(stack, output, adxl_rate, lsm_rate,
                                         fits, overrides, emit_rows=not use_native and not summary_only)
        bme_writers = {
            sid: csv_writer(stack, output / (NAMES[sid] + '.csv'), BME_COLUMNS)
            for sid in (5, 6)
        }
        audio_timing = csv_writer(stack, output / 'audio_packets.csv',
                                 ['packet_index', 'file_offset', 'dequeue_timestamp_us',
                                  'wav_first_frame', 'frames'])
        health_writer = csv_writer(stack, output / 'health_snapshots.csv',
                                   ['timestamp_us','sequence','reason','uptime_ms','sensor_id'] +
                                   SENSOR_FIELDS + ['sum_pause_call_us'] + GLOBAL_FIELDS +
                                   ['sd_requested_bytes','diag_failed_pushes','sd_block_bytes',
                                    'sd_admission_budget_us','lsm_proactive_interval_us','bench_sd_long_calls',
                                    'bench_sd_omitted','bench_adxl_omitted_main','bench_adxl_omitted_sat',
                                    'bench_sd_worst_start_us','bench_sd_worst_duration_us'])
        register_writer = csv_writer(stack, output / 'diagnostic_lsm_registers.csv',
                                     ['timestamp_us','sensor_id','address_hex','value_hex','read_ok'])
        adxl_trace_writer = csv_writer(stack, output / 'diagnostic_adxl_trace.csv',
            ['timestamp_us','sensor_id','sequence','start_us','discarded_words','entries_before',
             'entries_after_payload','flags','bad_raw_word','first9_ssi','last9_ssi','first_bad_word'])
        sd_trace_writer = csv_writer(stack, output / 'diagnostic_sd_trace.csv',
            ['timestamp_us','start_us','duration_us','requested_bytes','written_bytes','ring_used_bytes',
             'pending_mask','reason'])
        wav = None
        if not use_native and not summary_only:
            wav = stack.enter_context(wave.open(str(output / 'flight_audio.wav'), 'wb'))
            wav.setparams((2, 2, int(audio_rate), 0, 'NONE', 'not compressed'))
        binary = stack.enter_context(source.open('rb'))
        try:
            for offset, sensor, raw_time, payload in iter_packets(binary, resync, framing):
                timestamp = clock.unwrap(sensor, raw_time)
                if sensor in (8, 255):
                    termination = {'type': 'EOF' if sensor == 255 else 'PFM',
                                   'timestamp_us': timestamp, 'file_offset': offset}
                    break
                packet_index = counts[sensor]
                counts[sensor] += 1
                first_irq.setdefault(sensor, timestamp)
                last_irq[sensor] = timestamp
                if sensor in DETAIL_SIZES:
                    try:
                        if sensor == 0x30:
                            health = decode_health(payload)
                            last_health = health
                            health_snapshots.append({'sequence': health['sequence'],
                                                     'timestamp_us': timestamp,
                                                     'reason': health['reason'],
                                                     'uptime_ms': health['uptime_ms']})
                            g = health['system']
                            global_columns = GLOBAL_FIELDS + ['sd_requested_bytes','diag_failed_pushes',
                                'sd_block_bytes','sd_admission_budget_us','lsm_proactive_interval_us',
                                'bench_sd_long_calls','bench_sd_omitted','bench_adxl_omitted_main',
                                'bench_adxl_omitted_sat','bench_sd_worst_start_us','bench_sd_worst_duration_us']
                            for sid, v in health['sensors'].items():
                                health_writer.writerow([timestamp, health['sequence'],health['reason'],
                                    health['uptime_ms'],sid] + [v[k] for k in SENSOR_FIELDS] +
                                    [v['sum_pause_call_us']] + [g[k] for k in global_columns])
                        else:
                            row = decode_detail(sensor,payload)
                            detail_counts[DIAG_NAMES[sensor]] += 1
                            if sensor == 0x31:
                                addresses = [0x07,0x08,0x09,0x0a,0x0d,0x10,0x11,0x12,
                                             0x13,0x15,0x16,0x17,0x18,0x63]
                                values = bytes.fromhex(row['values_hex'])
                                for i, address in enumerate(addresses):
                                    register_writer.writerow([timestamp,row['sensor_id'],
                                        hex(address),hex(values[i]),int(bool(row['valid_mask'] & (1<<i)))])
                            elif sensor == 0x32:
                                adxl_trace_writer.writerow([timestamp] + [row.get(k) for k in
                                    ('sensor_id','sequence','start_us','discarded_words','entries_before',
                                     'entries_after_payload','flags','bad_raw_word','first9_ssi',
                                     'last9_ssi','first_bad_word')])
                            elif sensor == 0x33:
                                sd_trace_writer.writerow([timestamp] + [row.get(k) for k in
                                    ('start_us','duration_us','requested_bytes','written_bytes',
                                     'ring_used_bytes','pending_mask','reason')])
                    except (ValueError, struct.error) as exc:
                        raise ParseError(f'Invalid diagnostic payload at {offset}: {exc}') from exc
                    continue
                if sensor in HIGH_RATE_IDS:
                    acquisition.decode(sensor, timestamp, payload, offset)
                    continue
                if sensor in (5, 6):
                    bme_writers[sensor].writerow([timestamp, *struct.unpack('<fff', payload)])
                else:
                    if not use_native and not summary_only:
                        values = struct.unpack('<256h', payload)
                        stereo = [v for pair in zip(values[:128], values[128:]) for v in pair]
                        wav.writeframesraw(struct.pack('<256h', *stereo))
                    audio_timing.writerow([packet_index, offset, timestamp, audio_frame, 128])
                    audio_frame += 128
        except ParseError as exc:
            failure = str(exc)
    if use_native:
        _merge_native_integrity(output, acquisition.summary)
    summary = {
        'acquisition': acquisition.summary,
        'export_mode': 'summary_only' if summary_only else 'full',
        'diagnostics': {'snapshot_count': len(health_snapshots),
                        'snapshots': health_snapshots,
                        'latest': last_health,
                        'detail_counts': dict(detail_counts),
                        'has_final_snapshot': any(s['reason'] in (1, 2) for s in health_snapshots)},
        'input': str(source), 'status': 'parse_error' if failure else 'terminated' if termination else 'physical_eof_without_marker',
        'error': failure, 'termination': termination,
        'packet_counts': {NAMES[k]: v for k, v in counts.items()},
        'first_packet_timestamp_us': {NAMES[k]: v for k, v in first_irq.items()},
        'last_packet_timestamp_us': {NAMES[k]: v for k, v in last_irq.items()},
        'framing': dict(framing),
        'lsm_timing_request': lsm_timing, 'lsm_rate_overrides_hz': overrides,
        'lsm_fit_report': fit_report, 'lsm_fit_warning': fit_warning,
                'rates_hz': {'adxl_nominal': adxl_rate, 'lsm_nominal': lsm_rate, 'wav': audio_rate},
        'units': {'ADXL_g_xyz': 'g', 'LSM_acc_xyz': 'm/s^2', 'LSM_gyr_xyz': 'deg/s'},
        'warning': 'Timestamps are reconstructed estimates: inspect each LSM epoch model and fit report. Affine fits correct average rate against MCU time, not long-term nonlinear drift or exact cross-sensor phase. Short/rejected epochs use explicitly labelled nominal fallback unless overridden. ADXL timing remains nominal/IRQ-anchored. Audio timestamps are dequeue times; WAV concatenates saved blocks without inferred silence.'
    }
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    if failure:
        raise ParseError(f'{failure}. Partial outputs and summary retained in {output}')
    return summary


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('input', type=Path)
    cli.add_argument('--out', type=Path)
    cli.add_argument('--adxl-rate', type=float, default=5120)
    cli.add_argument('--lsm-rate', type=float, default=6670)
    cli.add_argument('--lsm-timing', choices=('auto','nominal'), default='auto', help='Auto fits acquisition epochs independently against MCU time; nominal disables fitting.')
    cli.add_argument('--lsm-rate-main', type=float, help='Explicit main rate override (Hz); takes precedence over auto.')
    cli.add_argument('--lsm-rate-sat', type=float, help='Explicit satellite rate override (Hz); takes precedence over auto.')
    cli.add_argument('--audio-rate' , type=int, default=44100)
    cli.add_argument('--resync', action='store_true', help='Scan for plausible headers after corrupt bytes; no CRC means recovery is heuristic.')
    cli.add_argument('--summary-only', action='store_true',
                     help='Quick integrity and health report without large sample CSVs/WAV; requires native C decoder.')
    cli.add_argument('--backend', choices=('auto', 'native', 'python'), default='auto',
                     help='Use optional compiled C decoder if available; Python remains fallback.')
    args = cli.parse_args()
    try:
        summary = parse_binary(args.input, args.out, args.adxl_rate, args.lsm_rate,
                               args.audio_rate, args.resync, args.lsm_timing,
                               args.lsm_rate_main, args.lsm_rate_sat, args.backend,
                               args.summary_only)
    except (ValueError, OSError) as exc:
        cli.exit(1, f'ERROR: {exc}\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
