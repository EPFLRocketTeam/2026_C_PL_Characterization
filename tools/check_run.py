#!/usr/bin/env python3
"""Inspect acquisition batches and run integrity, avoiding sample-CSV scans when
parser summary contains native integrity counters. No third-party dependencies.
"""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


class RateFit:
    """Constant-memory equivalent of linear fit over per-batch MCU timestamps."""
    __slots__ = ('origin_us', 'origin_index', 'n', 'mx', 'my', 'xx', 'xy', 'last_us')

    def __init__(self):
        self.origin_us = self.origin_index = self.last_us = None
        self.n = 0
        self.mx = self.my = self.xx = self.xy = 0.0

    def add(self, timestamp_us, sample_index):
        if self.origin_us is None:
            self.origin_us = timestamp_us
            self.origin_index = sample_index
        x = (timestamp_us-self.origin_us)/1e6
        y = sample_index-self.origin_index
        self.n += 1
        dx, dy = x-self.mx, y-self.my
        self.mx += dx/self.n
        self.my += dy/self.n
        self.xx += dx*(x-self.mx)
        self.xy += dx*(y-self.my)
        self.last_us = timestamp_us

    def result(self):
        if self.n < 3:
            return None
        span = (self.last_us-self.origin_us)/1e6
        if span < 1 or self.xx == 0:
            return None
        return {'rate_vs_mcu_hz': self.xy/self.xx, 'span_s': span, 'batches': self.n}


def legacy_continuity(root):
    """Fallback for older pure-Python outputs without native summary counters."""
    result = {}
    for loc in ('main', 'sat'):
        for kind in ('accel', 'gyro'):
            path = root/f'lsm_{loc}_{kind}.csv'
            if not path.exists():
                continue
            previous = None
            n = index_gaps = tag_steps = 0
            with path.open(newline='') as f:
                for row in csv.DictReader(f):
                    n += 1
                    if previous and row['epoch'] == previous['epoch']:
                        index_gaps += int(row['sample_index']) != int(previous['sample_index'])+1
                        tag_steps += (int(row['tag_counter'])-int(previous['tag_counter']))%4 != 1
                    previous = row
            result[path.name] = {'records': n, 'index_discontinuities': index_gaps,
                                 'unexpected_mod4_tag_steps': tag_steps}
    return result


def check(root):
    summary = json.loads((root/'summary.json').read_text())
    fits = defaultdict(lambda: {'accel': RateFit(), 'gyro': RateFit()})
    flags = defaultdict(Counter)
    counts = Counter()
    batch_sizes = defaultdict(Counter)
    with (root/'acquisition_batches.csv').open(newline='') as f:
        for row in csv.DictReader(f):
            sid, epoch = int(row['sensor_id']), int(row['epoch'])
            count, flag = int(row['record_count']), int(row['flags_hex'], 16)
            counts[sid] += count
            flags[sid][hex(flag)] += 1
            batch_sizes[sid][count] += 1
            if sid in (19,20) and count and not flag & 0xde:
                r = fits[sid, epoch]
                timestamp = int(row['read_start_us'])
                r['accel'].add(timestamp,int(row['first_acc_index']))
                r['gyro'].add(timestamp,int(row['first_gyro_index']))
    rates = [{'sensor_id':sid,'epoch':epoch,
              'accel':v['accel'].result(), 'gyro':v['gyro'].result()}
             for (sid,epoch),v in sorted(fits.items())]
    acq = summary.get('acquisition', {})
    continuity = acq.get('lsm_continuity')
    if continuity is None:
        continuity = legacy_continuity(root)
    return {'parse_status': summary['status'], 'parse_error': summary['error'],
            'records': dict(counts), 'flags': {k: dict(v) for k,v in flags.items()},
            'batch_sizes': {k: dict(v) for k,v in batch_sizes.items()},
            'lsm_epoch_rate_estimates': rates, 'lsm_continuity': continuity,
            'parser_lsm_timing_models': acq.get('lsm_timing_models',[]),
            'parser_lsm_fit_warning': summary.get('lsm_fit_warning'),
            'missing_batch_sequences': acq.get('missing_batch_sequences',{}),
            'notes': ['Rate estimates use sample indices versus MCU service-start times; FIFO backlog adds uncertainty.',
                      'Do not infer exact loss fractions from nominal ODR ratios.',
                      'Modulo-four tags cannot detect missing multiples of four time slots.',
                      'Inspect parser_lsm_timing_models for fitted, user-rate, or nominal CSV timing; older parsers omit this field.',
                      'Affine timing fits correct average rate only; clock phase, filter delay and nonlinear drift remain uncertain.',
                      'Serial SD trace timestamps wrap at 2^32 microseconds. Align them before comparing long recordings.']}


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('parsed_directory', type=Path)
    cli.add_argument('--out', type=Path)
    args = cli.parse_args()
    result = json.dumps(check(args.parsed_directory), indent=2)
    if args.out:
        args.out.write_text(result+'\n')
    print(result)
