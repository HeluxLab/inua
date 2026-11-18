# Copia en Python de RoomLogic (components/cubo_room/cubo_room.h) para probar la regla de decision
# con senales sinteticas: python tools/sim_room.py. Si se cambia la regla en el .h, cambiarla aca tambien.
import random, sys

class Beacon:
    def __init__(self, room, offset=0):
        self.room = room; self.offset = offset; self.rssi = -127.0; self.last_seen = 0; self.hits = 0

class RoomLogic:
    def __init__(self, alpha=0.25, hysteresis=6.0, min_rssi=-85.0, timeout_ms=30000, settle_ms=3000, dwell_ms=1500):
        self.alpha = alpha; self.hysteresis = hysteresis; self.min_rssi = min_rssi
        self.timeout_ms = timeout_ms; self.settle_ms = settle_ms; self.dwell_ms = dwell_ms
        self.beacons = []; self.room = -1; self.current = -1; self.last_alive = 0
        self.challenger = -1; self.challenger_since = 0

    def hit(self, i, rssi, now):
        b = self.beacons[i]; r = rssi + b.offset
        if b.hits == 0 or now - b.last_seen > self.timeout_ms:
            b.rssi = r; b.hits = 1
        else:
            b.rssi += self.alpha * (r - b.rssi); b.hits = min(b.hits + 1, 0xFFFF)
        b.last_seen = now

    def evaluate(self, now):
        if now < self.settle_ms:
            return False
        best = cur = -1
        for i, b in enumerate(self.beacons):
            if b.hits == 0:
                continue
            if now - b.last_seen > self.timeout_ms:
                b.hits = 0; continue
            if b.rssi >= self.min_rssi and (best < 0 or b.rssi > self.beacons[best].rssi):
                best = i
            if self.room >= 0 and b.room == self.room and (cur < 0 or b.rssi > self.beacons[cur].rssi):
                cur = i
        self.current = cur
        if best < 0 and cur < 0:
            self.challenger = -1
            if self.room >= 0 and now - self.last_alive > self.timeout_ms:
                self.room = -1; return True
            return False
        self.last_alive = now
        if best < 0:
            self.challenger = -1; return False
        b = self.beacons[best]
        if b.room == self.room:
            self.challenger = -1; return False
        if cur >= 0:
            if not (b.hits >= 2 and b.rssi >= self.beacons[cur].rssi + self.hysteresis):
                self.challenger = -1; return False
            if self.challenger != b.room:
                self.challenger = b.room; self.challenger_since = now
            if now - self.challenger_since < self.dwell_ms:
                return False
        self.room = b.room; self.current = best; self.challenger = -1
        return True

def run(levels, seconds, sigma=4.0, adv_ms=100, duty=0.25, restored=-1, seed=1, **kw):
    """levels(t_ms) -> list of mean RSSI per beacon (None = not heard). One beacon per room."""
    rnd = random.Random(seed)
    lg = RoomLogic(**kw)
    n = len(levels(0))
    lg.beacons = [Beacon(i) for i in range(n)]
    lg.room = restored
    changes = []
    nxt = [rnd.uniform(0, adv_ms) for _ in range(n)]
    for t in range(0, int(seconds * 1000), 10):
        lv = levels(t)
        for i in range(n):
            if t >= nxt[i]:
                nxt[i] += adv_ms + rnd.uniform(0, 10)
                if lv[i] is not None and rnd.random() < duty:
                    lg.hit(i, int(round(rnd.gauss(lv[i], sigma))), t)
        if t % 250 == 0 and lg.evaluate(t):
            changes.append((t / 1000.0, lg.room))
    return changes

def flips_per_hour(diff, **kw):
    tot = 0
    for seed in range(5):
        ch = run(lambda t: [-70.0, -70.0 + diff], 3600, seed=seed, restored=0, **kw)
        tot += len(ch)
    return tot / 5.0

if __name__ == '__main__':
    for name, kw in [('alpha .35, no dwell', dict(alpha=0.35, dwell_ms=0)),
                     ('alpha .35, dwell 1.5 s', dict(alpha=0.35, dwell_ms=1500)),
                     ('alpha .25, dwell 1.5 s', dict(alpha=0.25, dwell_ms=1500)),
                     ('alpha .25, dwell 2 s', dict(alpha=0.25, dwell_ms=2000)),
                     ('alpha .20, dwell 2 s', dict(alpha=0.20, dwell_ms=2000))]:
        print('==', name)
        print('  cold boot A -55 / B -75      :', run(lambda t: [-55.0, -75.0], 20, **kw))
        print('  wake restored A, same levels :', run(lambda t: [-55.0, -75.0], 20, restored=0, **kw))
        print('  wake restored B, cube in A   :', run(lambda t: [-55.0, -75.0], 20, restored=1, **kw))
        walk = lambda t: [-55.0 - 30.0 * min(1, max(0, (t - 5000) / 8000.0)), -85.0 + 30.0 * min(1, max(0, (t - 5000) / 8000.0))]
        print('  walk A to B from t=5 to 13 s :', run(walk, 30, restored=0, **kw))
        print('  A unplugged at 10 s, B -75   :', run(lambda t: [None if t > 10000 else -55.0, -75.0], 80, restored=0, **kw))
        print('  all beacons gone at 10 s     :', run(lambda t: [None if t > 10000 else -55.0, None if t > 10000 else -75.0], 120, restored=0, **kw))
        print('  only weak own beacon (-90)   :', run(lambda t: [-90.0, None], 120, restored=0, **kw))
        for d in (0, 2, 4):
            print('  boundary, B stronger by %d dB, sigma 4: %.1f changes/hour' % (d, flips_per_hour(d, **kw)),
                  '| sigma 2: %.1f' % flips_per_hour(d, sigma=2.0, **kw))
        print('  slow beacons (1 s), walk A to B:', run(walk, 40, restored=0, adv_ms=1000, **kw))
