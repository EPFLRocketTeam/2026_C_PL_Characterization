"""Diagnostic V1 binary packets. Kept in sync with src/diagnostics.cpp.
Do not reorder fields without changing the packet version.
"""
import struct
SENSOR_FIELDS = ['interrupts', 'overwritten_irq_timestamps', 'services', 'skipped_services', 'empty_services', 'pause_calls', 'level_services', 'records_read', 'records_queued', 'fifo_overruns', 'fifo_resets', 'fifo_recovery_failures', 'unknown_tags', 'temp_read_errors', 'max_service_latency_us', 'queued_packets', 'ring_drops', 'too_few_entries', 'invalid_fifo_request', 'marker_failures', 'drain_failures', 'discarded_words', 'incomplete_lsm_batches', 'lsm_io_errors', 'min_entries', 'max_entries', 'entries_eq_299', 'entries_ge_510', 'max_pause_call_us']
GLOBAL_FIELDS = ['ring_high_water_bytes', 'ring_drop_packets', 'sd_write_calls', 'sd_short_writes', 'sd_max_write_us', 'sd_close_failures', 'sd_headroom_cache_hits', 'sd_headroom_cooldown_skips', 'sd_normal_write_calls', 'sd_normal_max_write_us', 'sd_normal_budget_exceeded', 'sd_headroom_probes', 'sd_headroom_drains', 'sd_headroom_deferrals', 'sd_forced_low_headroom', 'sd_busy_skips', 'sd_deferred_calls', 'sd_forced_pending_writes', 'sd_max_defer_us']
HEALTH_SIZE = 12 + 7*(len(SENSOR_FIELDS)*4+8) + len(GLOBAL_FIELDS)*4+8+16+24
DETAIL_SIZES = {0x30: HEALTH_SIZE, 0x31: 18, 0x32: 28, 0x33: 24}
DIAG_NAMES = {0x30: 'diagnostic_health', 0x31: 'diagnostic_lsm_registers',
              0x32: 'diagnostic_adxl_trace', 0x33: 'diagnostic_sd_trace'}
def decode_health(payload):
    if len(payload) != HEALTH_SIZE or payload[0] != 1 or payload[2] != 7 or payload[3] != len(SENSOR_FIELDS):
        raise ValueError('Invalid diagnostic health version, length, or field count')
    version, reason, n_sensors, n_fields, sequence, uptime_ms = struct.unpack_from('<BBBBII', payload)
    offset = 12
    sensors = {}
    for sid in range(1,8):
        vals = struct.unpack_from('<'+ 'I'*len(SENSOR_FIELDS)+'Q',payload,offset)
        offset += len(SENSOR_FIELDS)*4+8
        sensors[str(sid)] = dict(zip(SENSOR_FIELDS+['sum_pause_call_us'],vals))
    vals = struct.unpack_from('<'+'I'*len(GLOBAL_FIELDS)+'Q'+'I'*10,payload,offset)
    system = dict(zip(GLOBAL_FIELDS+['sd_requested_bytes','diag_failed_pushes',
                                     'sd_block_bytes','sd_admission_budget_us','lsm_proactive_interval_us', 'bench_sd_long_calls','bench_sd_omitted',
             'bench_adxl_omitted_main','bench_adxl_omitted_sat','bench_sd_worst_start_us',
             'bench_sd_worst_duration_us'],vals))
    return {'version': version,'reason':reason,'sequence':sequence, 'uptime_ms':uptime_ms,
            'sensors':sensors,'system':system}
def decode_detail(sensor,payload):
    if not payload or payload[0] != 1 or len(payload) != DETAIL_SIZES[sensor]:
        raise ValueError('Invalid diagnostic detail version/length')
    if sensor == 0x31:
        version,sid,mask=struct.unpack_from('<BBH',payload)
        return {'sensor_id':sid,'valid_mask':mask,'values_hex':payload[4:].hex()}
    if sensor == 0x32:
        fields=('sequence','start_us','discarded_words','entries_before',
                'entries_after_payload','flags','bad_raw_word','first9_ssi',
                'last9_ssi','first_bad_word')
        vals=struct.unpack_from('<IIIHHHHHHh',payload,2)
        d=dict(zip(fields,vals));d['sensor_id']=payload[1]
        if d['entries_after_payload']==65535:d['entries_after_payload']=None
        return d
    if sensor == 0x33:
        vals=struct.unpack_from('<IIIII',payload,4)
        return dict(zip(('start_us','duration_us','requested_bytes','written_bytes','ring_used_bytes'),vals),
                    pending_mask=payload[1],reason=payload[2])
    raise ValueError('Unsupported diagnostic ID')
