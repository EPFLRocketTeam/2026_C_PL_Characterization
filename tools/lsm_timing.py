"""Bounded-memory affine LSM timing estimates, independently for each reset epoch.
Inputs are acquisition batch metadata and unwrapped MCU service times, not CSV sample times.
No hardware-timestamp synchronization is implied.
"""
import math


class EpochFit:
    def __init__(self):
        self.n = 0
        self.mx = self.my = self.xx = self.xy = self.yy = 0.0
        self.origin_us = self.origin_index = None
        self.last_us = None

    def add(self, service_us, first_index, fifo_entries):
        if self.origin_us is None:
            self.origin_us, self.origin_index = service_us, first_index
        self.last_us = service_us
        x = (service_us-self.origin_us)/1e6
        # Both streams have the same configured ODR. Their relative phase and
        # the odd-record case remain uncertain by about one sample.
        y = first_index-self.origin_index + max(0, (fifo_entries+1)//2-1)
        self.n += 1
        dx, dy = x-self.mx, y-self.my
        self.mx += dx/self.n
        self.my += dy/self.n
        self.xx += dx*(x-self.mx)
        self.xy += dx*(y-self.my)
        self.yy += dy*(y-self.my)

    def finish(self, reference_rate):
        span = 0 if self.last_us is None else (self.last_us-self.origin_us)/1e6
        result = {'batches': self.n, 'span_s': span, 'accepted': False}
        if self.n < 30 or span < 5 or self.xx <= 0:
            result['reason'] = 'needs at least 30 valid batches spanning 5 seconds'
            return result
        rate = self.xy/self.xx
        if not math.isfinite(rate) or not .8*reference_rate <= rate <= 1.2*reference_rate:
            result['reason'] = 'fitted rate outside 80–120% of reference rate'
            return result
        offset_samples = self.my-rate*self.mx
        base = self.origin_us-(self.origin_index+offset_samples)*1e6/rate
        residual_samples = math.sqrt(max(0.0, self.yy-rate*self.xy)/self.n)
        rms_us = residual_samples*1e6/rate
        result.update(rate_hz=rate, base_us=base, residual_rms_us=rms_us)
        if rms_us > 2000:
            result['reason'] = 'FIFO/service-time fit residual exceeds 2 ms RMS'
            return result
        result.update(accepted=True, reason='affine fit against MCU time with FIFO-level correction')
        return result


class TimingEstimator:
    def __init__(self):
        self.fits = {}
        self.invalid = set()
        self.last = {}

    def add(self, sensor, epoch, service_us, acc_index, gyro_index, entries, count):
        key = sensor, epoch
        prev = self.last.get(key)
        if prev and (service_us < prev[0] or acc_index < prev[1] or gyro_index < prev[2]):
            self.invalid.add(key)
        self.last[key] = service_us, acc_index, gyro_index
        if not count or not count <= entries <= 512:
            return
        for kind, index in [('accel', acc_index), ('gyro', gyro_index)]:
            self.fits.setdefault((sensor, epoch, kind), EpochFit()).add(service_us, index, entries)

    def finish(self, reference_rate):
        models, report = {}, []
        epochs = sorted(set((s,e) for s,e,k in self.fits))
        for sensor, epoch in epochs:
            fits = {k:self.fits[sensor,epoch,k].finish(reference_rate) for k in ('accel','gyro')}
            if (sensor,epoch) in self.invalid:
                for f in fits.values(): f.update(accepted=False, reason='nonmonotonic sample index/time')
            accepted = all(f['accepted'] for f in fits.values())
            if accepted:
                a,g = fits['accel']['rate_hz'],fits['gyro']['rate_hz']
                if abs(a-g)/((a+g)/2) > .005:
                    accepted = False
                    for f in fits.values(): f.update(accepted=False, reason='accel/gyro rates differ by more than 0.5%')
            if accepted: models[sensor,epoch] = fits
            report.append({'sensor_id':sensor,'epoch':epoch,'accepted':accepted,**fits})
        return models, report
