#!/usr/bin/env python3
"""ASCII STL -> parts.js, with a collision-verified assembly plan.

For every part we know the axis it goes in on (or the pin it rides on). The
planner walks the assembly in gunsmith order, voxelises the geometry that is
already installed, and pushes the incoming part out along its axis until it is
clear. It then sweeps the whole approach at voxel resolution and reports any
remaining interpenetration, so every path in the output is one the part can
physically travel without passing through another part.

Run:  python3 build.py
"""
import base64, glob, json, os, pickle, struct, sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = (os.path.dirname(HERE) if os.path.isdir(os.path.join(os.path.dirname(HERE), 'Glock 17.STL'))
       else os.path.expanduser('~/Glock17-3D-print-model'))
VOX = 0.03          # voxel edge, inches
CONTACT = 6         # extra buried voxels tolerated beyond the seated state

# id, file, label, material, group, dir
# `dir` points from the seated position out to where the part waits before it
# goes in - i.e. the part travels -dir to seat. Chosen from the real opening.
PARTS = [
    ('frame',    'Glock 17 - Glock_frame-1.STL',                        'Frame',                       'polymer', 'frame', (0, 0, 0)),
    ('housing',  'Glock 17 - Glock_trigger mech housing-1.STL',         'Trigger mechanism housing',   'polymer', 'frame', (0, -1, 0)),
    ('tbar',     'Glock 17 - Glock_trigger bar-1.STL',                  'Trigger bar',                 'steel',   'frame', (0, 1, 0)),
    ('connector','Glock 17 - Glock_connector-1.STL',                    'Connector',                   'steel',   'frame', (0, 1, 0)),
    ('tmpin',    'Glock 17 - Glock_trigger mech pin-1.STL',             'Trigger housing pin',         'steel',   'frame', (0, 0, 1)),
    ('trigger',  'Glock 17 - Glock_trigger-1.STL',                      'Trigger',                     'polymer', 'frame', (0, -1, 0)),
    ('tsafety',  'Glock 17 - Glock_trigger safety-1.STL',               'Trigger safety',              'polymer', 'frame', (1, 0, 0)),
    ('tsp',      'Glock 17 - Glock_trigger safety pin-1.STL',           'Trigger safety pin',          'steel',   'frame', (0, 0, 1)),
    ('tpin',     'Glock 17 - Glock_trigger pin-1.STL',                  'Trigger pin',                 'steel',   'frame', (0, 0, 1)),
    ('magcatch', 'Glock 17 - Glock_mag catch-1.STL',                    'Magazine catch',              'polymer', 'frame', (0, 0, -1)),
    ('slock',    'Glock 17 - Glock_slide lock-1.STL',                   'Slide lock',                  'steel',   'frame', (0, 0, 1)),
    ('scatch',   'Glock 17 - Glock_slide catch-1.STL',                  'Slide catch lever',           'steel',   'frame', (0, 0, -1)),
    ('lockblock','Glock 17 - Glock_locking block-1.STL',                'Locking block',               'steel',   'frame', (0, 1, 0)),
    ('lbpin',    'Glock 17 - Glock_locking block pin-1.STL',            'Locking block pin',           'steel',   'frame', (0, 0, 1)),
    ('firepin',  'Glock 17 - Glock_fire pin-1.STL',                     'Firing pin',                  'steel',   'slide', (-1, 0, 0)),
    ('sprgcup',  'Glock 17 - Glock_sprg cup-1.STL',                     'Firing pin spring cup',       'steel',   'slide', (-1, 0, 0)),
    ('sleeve',   'Glock 17 - Glock_spacer sleeve-1.STL',                'Spacer sleeve',               'steel',   'slide', (-1, 0, 0)),
    ('fpsafety', 'Glock 17 - Glock_fire pin safety-1.STL',              'Firing pin safety',           'steel',   'slide', (0, 1, 0)),
    ('bearing',  'Glock 17 - Glock_extractor sprg bearing-1.STL',       'Extractor spring bearing',    'steel',   'slide', (-1, 0, 0)),
    ('plunger',  'Glock 17 - Glock_extractor depressor plunger-1.STL',  'Extractor depressor plunger', 'steel',   'slide', (-1, 0, 0)),
    ('extractor','Glock 17 - Glock_extractor-1.STL',                    'Extractor',                   'steel',   'slide', (0, 1, 0)),
    ('scover',   'Glock 17 - Glock_slide cover-1.STL',                  'Slide cover plate',           'steel',   'slide', (0, -1, 0)),
    ('fsight',   'Glock 17 - Glock_front sight-1.STL',                  'Front sight',                 'steel',   'slide', (0, 1, 0)),
    ('rsight',   'Glock 17 - Glock_rear sight-1.STL',                   'Rear sight',                  'steel',   'slide', (0, 0, 1)),
    ('barrel',   'Glock 17 - Glock_barrel-2.STL',                       'Barrel',                      'steel',   'slide', (1, 0, 0)),
    ('recoil',   'Glock 17 - Glock_recoil sprg guide-1.STL',            'Recoil spring assembly',      'steel',   'slide', (1, 0, 0)),
    ('slide',    'Glock 17 - Glock_slide-1.STL',                        'Slide',                       'steel',   'slide', (1, 0, 0)),
    ('follower', 'Glock 17 - Glock_Magazine-1 Glock_mag follower-1.STL','Magazine follower',           'polymer', 'mag',   (0, 1, 0)),
    ('insert',   'Glock 17 - Glock_Magazine-1 Glock_mag insert-1.STL',  'Magazine insert',             'polymer', 'mag',   (0, -1, 0)),
    ('plate',    'Glock 17 - Glock_Magazine-1 Glock_mag plate-1.STL',   'Magazine baseplate',          'polymer', 'mag',   (0, -1, 0)),
    ('mag',      'Glock 17 - Glock_Magazine-1 Glock_mag-1.STL',         'Magazine body',               'polymer', 'mag',   (0, -1, 0)),
]
LABEL = {p[0]: p[2] for p in PARTS}

# gunsmith order: each entry is one motion, one or more parts moving together
STEPS = [
    ('Trigger mechanism housing into the frame', ['housing']),
    ('Trigger bar into the housing',             ['tbar']),
    ('Connector onto the trigger bar',           ['connector']),
    ('Trigger housing pin',                      ['tmpin']),
    ('Trigger into the frame',                   ['trigger']),
    ('Trigger safety onto the trigger',          ['tsafety']),
    ('Trigger safety pin',                       ['tsp']),
    ('Trigger pin',                              ['tpin']),
    ('Magazine catch',                           ['magcatch']),
    ('Slide lock',                               ['slock']),
    ('Slide catch lever',                        ['scatch']),
    ('Locking block',                            ['lockblock']),
    ('Locking block pin',                        ['lbpin']),
    ('Firing pin, spring cup and spacer sleeve', ['firepin', 'sprgcup', 'sleeve']),
    ('Firing pin safety',                        ['fpsafety']),
    ('Extractor spring bearing and plunger',     ['bearing', 'plunger']),
    ('Extractor',                                ['extractor']),
    ('Slide cover plate',                        ['scover']),
    ('Barrel into the slide',                    ['barrel']),
    ('Recoil spring assembly',                   ['recoil']),
    ('Front sight',                              ['fsight']),
    ('Rear sight',                               ['rsight']),
    ('Slide onto the frame',                     ['slide']),
    ('Magazine follower',                        ['follower']),
    ('Magazine insert',                          ['insert']),
    ('Magazine baseplate',                       ['plate']),
    ('Magazine into the frame',                  ['mag']),
]

# where each subassembly sits while its own parts are fitted
GROUP_BENCH = {'frame': (0, 0, 0), 'slide': (7.6, 0, 0), 'mag': (0, -3.4, 0)}
GROUP_INSTALL_STEP = {'slide': 'Slide onto the frame', 'mag': 'Magazine into the frame'}
GROUP_DIR = {'slide': (1, 0, 0), 'mag': (0, -1, 0)}
GROUP_EXTRA = {}

TRI = struct.Struct('<3f')
CACHE = os.path.join(HERE, '.voxcache.pkl')


# ---------------------------------------------------------------- geometry
def read_ascii_stl(path):
    data = open(path, 'rb').read()
    out = bytearray()
    mn = [1e30] * 3
    mx = [-1e30] * 3
    n = 0
    for chunk in data.split(b'facet normal')[1:]:
        verts = [l.split() for l in chunk.split(b'\n') if l.strip().startswith(b'vertex')]
        if len(verts) != 3:
            continue
        for v in verts:
            x, y, z = float(v[1]), float(v[2]), float(v[3])
            out += TRI.pack(x, y, z)
            n += 1
            for i, c in enumerate((x, y, z)):
                if c < mn[i]:
                    mn[i] = c
                if c > mx[i]:
                    mx[i] = c
    return bytes(out), n // 3, mn, mx


def load_all():
    cache = {}
    if os.path.exists(CACHE):
        cache = pickle.load(open(CACHE, 'rb'))
    parts = []
    for pid, fname, label, cat, group, axis in PARTS:
        path = os.path.join(SRC, fname)
        if not os.path.exists(path):
            raise SystemExit('missing ' + path)
        data, tris, mn, mx = read_ascii_stl(path)
        key = (fname, os.path.getmtime(path), VOX)
        if key not in cache:
            cache[key] = solidify(voxelise(data))
        parts.append(dict(id=pid, label=label, cat=cat, group=group, axis=axis,
                          data=data, tris=tris, min=mn, max=mx, vox=cache[key]))
    pickle.dump(cache, open(CACHE, 'wb'))
    return parts


# ---------------------------------------------------------------- voxels
def voxelise(buf):
    """Surface voxels of a closed mesh, dilating each triangle by half a cell."""
    cells = set()
    r = VOX * 0.5
    n = len(buf) // 36
    for t in range(n):
        o = t * 36
        ax, ay, az = TRI.unpack_from(buf, o)
        bx, by, bz = TRI.unpack_from(buf, o + 12)
        cx, cy, cz = TRI.unpack_from(buf, o + 24)
        # dominant axis -> project to the other two
        ex, ey, ez = bx - ax, by - ay, bz - az
        fx, fy, fz = cx - ax, cy - ay, cz - az
        nx, ny, nz = ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx
        if nx == 0 and ny == 0 and nz == 0:
            continue
        nl = (nx * nx + ny * ny + nz * nz) ** 0.5
        nx, ny, nz = nx / nl, ny / nl, nz / nl
        if abs(nx) >= abs(ny) and abs(nx) >= abs(nz):
            u, v = (1, 2)
        elif abs(ny) >= abs(nz):
            u, v = (0, 2)
        else:
            u, v = (0, 1)
        a = (ax, ay, az); b = (bx, by, bz); c = (cx, cy, cz)
        lo = [int(min(a[i], b[i], c[i]) / VOX - 1.0) for i in range(3)]
        hi = [int(max(a[i], b[i], c[i]) / VOX + 1.0) for i in range(3)]
        # barycentric in the projected plane
        Ax, Ay = a[u], a[v]
        Bx, By = b[u], b[v]
        Cx, Cy = c[u], c[v]
        den = (By - Cy) * (Ax - Cx) + (Cx - Bx) * (Ay - Cy)
        if abs(den) < 1e-12:
            continue
        for i in range(lo[0], hi[0] + 1):
            for j in range(lo[1], hi[1] + 1):
                for k in range(lo[2], hi[2] + 1):
                    px, py, pz = i * VOX, j * VOX, k * VOX
                    pd = (px - ax) * nx + (py - ay) * ny + (pz - az) * nz
                    if abs(pd) > r * 1.7:
                        continue
                    Px = (px, py, pz)[u]
                    Py = (px, py, pz)[v]
                    w1 = ((By - Cy) * (Px - Cx) + (Cx - Bx) * (Py - Cy)) / den
                    w2 = ((Cy - Ay) * (Px - Cx) + (Ax - Cx) * (Py - Cy)) / den
                    w3 = 1.0 - w1 - w2
                    m = -0.45
                    if w1 >= m and w2 >= m and w3 >= m:
                        cells.add((i, j, k))
    return cells


def solidify(surface):
    """Fill a closed surface into a solid so touches are not read as collisions
    and through-holes stay open."""
    if not surface:
        return surface
    it = iter(surface)
    i, j, k = next(it)
    mn = [i, j, k]; mx = [i, j, k]
    for (i, j, k) in it:
        if i < mn[0]: mn[0] = i
        if j < mn[1]: mn[1] = j
        if k < mn[2]: mn[2] = k
        if i > mx[0]: mx[0] = i
        if j > mx[1]: mx[1] = j
        if k > mx[2]: mx[2] = k
    mn = [mn[0] - 1, mn[1] - 1, mn[2] - 1]
    mx = [mx[0] + 1, mx[1] + 1, mx[2] + 1]
    w = mx[0] - mn[0] + 1
    h = mx[1] - mn[1] + 1
    d = mx[2] - mn[2] + 1
    seen = bytearray(w * h * d)
    for (i, j, k) in surface:
        seen[(i - mn[0]) + w * ((j - mn[1]) + h * (k - mn[2]))] = 1
    q = deque()
    for i in range(w):
        for j in range(h):
            for k in (0, d - 1):
                q.append((i, j, k))
    for i in range(w):
        for k in range(d):
            for j in (0, h - 1):
                q.append((i, j, k))
    for j in range(h):
        for k in range(d):
            for i in (0, w - 1):
                q.append((i, j, k))
    while q:
        i, j, k = q.popleft()
        idx = i + w * (j + h * k)
        if seen[idx]:
            continue
        seen[idx] = 1
        if i > 0: q.append((i - 1, j, k))
        if i + 1 < w: q.append((i + 1, j, k))
        if j > 0: q.append((i, j - 1, k))
        if j + 1 < h: q.append((i, j + 1, k))
        if k > 0: q.append((i, j, k - 1))
        if k + 1 < d: q.append((i, j, k + 1))
    for i in range(w):
        for j in range(h):
            for k in range(d):
                if not seen[i + w * (j + h * k)]:
                    surface.add((i + mn[0], j + mn[1], k + mn[2]))
    return surface


def shift(cells, d):
    di, dj, dk = d
    return {(i + di, j + dj, k + dk) for (i, j, k) in cells}


def bbox_of(cells):
    it = iter(cells)
    i, j, k = next(it)
    mn = [i, j, k]; mx = [i, j, k]
    for (i, j, k) in it:
        if i < mn[0]: mn[0] = i
        if j < mn[1]: mn[1] = j
        if k < mn[2]: mn[2] = k
        if i > mx[0]: mx[0] = i
        if j > mx[1]: mx[1] = j
        if k > mx[2]: mx[2] = k
    return mn, mx


def overlap_count(cells, off, other):
    """Depth of interpenetration, not contact area: count only cells whose six
    neighbours are also inside the intersection. Two surfaces sliding past each
    other with clearance produce a one-cell-thick band and score zero."""
    di, dj, dk = off
    hit = set()
    for (i, j, k) in cells:
        c = (i + di, j + dj, k + dk)
        if c in other:
            hit.add(c)
    if not hit:
        return 0
    core = 0
    for (i, j, k) in hit:
        if ((i + 1, j, k) in hit and (i - 1, j, k) in hit
                and (i, j + 1, k) in hit and (i, j - 1, k) in hit
                and (i, j, k + 1) in hit and (i, j, k - 1) in hit):
            core += 1
    return core


def box_of(cells):
    mn, mx = bbox_of(cells)
    return (mn[0], mn[1], mn[2], mx[0], mx[1], mx[2])


def bbox_hit(a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5):
    return not (a3 < b0 or a0 > b3 or a4 < b1 or a1 > b4 or a5 < b2 or a2 > b5)


# ---------------------------------------------------------------- planning
def plan(parts):
    byId = {p['id']: p for p in parts}
    for p in parts:
        p['vbox'] = box_of(p['vox'])
        p['bench'] = [int(round(v / VOX)) for v in GROUP_BENCH[p['group']]]
        p['world'] = shift(p['vox'], p['bench'])
        p['wbox'] = box_of(p['world'])
        p['staged'] = [0.0, 0.0, 0.0]

    def group_of(pid):
        return byId[pid]['group']

    def union_of(ids, off):
        u = set()
        for pid in ids:
            b = byId[pid]['bench']
            u |= shift(byId[pid]['vox'], (b[0] + off[0], b[1] + off[1], b[2] + off[2]))
        return u

    def ubox(ids, off):
        boxes = []
        for pid in ids:
            b = byId[pid]['bench']
            w = shift(byId[pid]['vox'], (b[0] + off[0], b[1] + off[1], b[2] + off[2]))
            boxes.append(box_of(w))
        return (min(b[0] for b in boxes), min(b[1] for b in boxes), min(b[2] for b in boxes),
                max(b[3] for b in boxes), max(b[4] for b in boxes), max(b[5] for b in boxes))

    # the frame is the fixture, and the slide and magazine bodies are what
    # their own subassemblies get built into - all three are present from the start
    installed = {k: byId[k]['world'] for k in ('frame', 'slide', 'mag')}
    stagger = {}
    step_of = {pid: i for i, (l, ids) in enumerate(STEPS) for pid in ids}
    g_of = {p['id']: p['group'] for p in parts}
    report = []
    for si, (label, ids) in enumerate(STEPS):
        group_move = label in GROUP_INSTALL_STEP.values()
        if group_move:
            g = group_of(ids[0])
            movers = [p['id'] for p in parts if p['group'] == g]
            axes = [GROUP_DIR[g]]
        else:
            movers = list(ids)
            axes = [byId[movers[0]]['axis']]
            for m in movers[1:]:
                assert tuple(byId[m]['axis']) == tuple(axes[0]), (m, 'axis differs inside one step')
        obst = [(q, installed[q]) for q in installed if q not in movers]
        mb = ubox(movers, (0, 0, 0))
        obst = [(q, s) for (q, s) in obst if bbox_hit(*mb, *byId[q]['wbox'])]

        seat = union_of(movers, (0, 0, 0))
        base = {q: overlap_count(seat, (0, 0, 0), cells) for (q, cells) in obst}

        def worst(off):
            u = union_of(movers, off)
            w = 0
            for (q, cells) in obst:
                c = overlap_count(u, (0, 0, 0), cells) - base[q]
                if c > w:
                    w = c
            return w

        # try the declared direction first, then the remaining five; a part that
        # cannot travel down its nominal axis gets the one that actually works
        if not group_move:
            primary = [(0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0)]
            diag = []
            for x in (-1, 0, 1):
                for y in (-1, 0, 1):
                    for z in (-1, 0, 1):
                        if (x, y, z) != (0, 0, 0) and (x, y, z) not in primary:
                            diag.append((x, y, z))
            for c in primary + diag:
                if tuple(c) not in [tuple(a) for a in axes]:
                    axes.append(c)
        chosen = None
        for ax in axes:
            a = list(ax)
            bucket = (g_of[movers[0]], tuple(a))
            seen = stagger.get(bucket, 0)
            d = int(round((1.7 + 0.62 * seen) / VOX))
            limit = int(14.0 / VOX)
            while d < limit and worst((a[0] * d, a[1] * d, a[2] * d)) > CONTACT:
                d += 1
            peak, bad = 0, None
            for st in range(d, -1, -1):
                w = worst((a[0] * st, a[1] * st, a[2] * st))
                if w > peak:
                    peak = w
                if w > CONTACT and bad is None:
                    bad = st
            if chosen is None or peak < chosen[0]:
                chosen = (peak, bad, a, d)
            if bad is None:
                break
        peak, bad, a, d = chosen
        stagger[(g_of[movers[0]], tuple(a))] = stagger.get((g_of[movers[0]], tuple(a)), 0) + 1
        standoff = d * VOX
        if bad is not None:
            report.append((movers[0], label,
                           'BLOCKED at %.2f in (+%d vox) axis %s' % (bad * VOX, peak, tuple(a)), standoff))
        else:
            note = '' if tuple(a) == tuple(axes[0]) else '  axis %s' % (tuple(a),)
            report.append((movers[0], label, 'clear  (max +%d vox)%s' % (peak, note), standoff))
        if group_move:
            GROUP_EXTRA[g_of[movers[0]]] = [a[0] * standoff, a[1] * standoff, a[2] * standoff]
        for m in movers:
            if not group_move:
                byId[m]['staged'] = [a[0] * standoff, a[1] * standoff, a[2] * standoff]
            elif step_of.get(m) == si:      # the bodies ride the group, no approach of their own
                byId[m]['staged'] = [0.0, 0.0, 0.0]
            installed[m] = byId[m]['world']
    return report


def calibrate(parts):
    """Known-bad placements, to prove the test separates a graze from a part
    sitting inside another part."""
    byId = {p['id']: p for p in parts}
    for p in parts:
        p['bench'] = [int(round(v / VOX)) for v in GROUP_BENCH[p['group']]]
        p['world'] = shift(p['vox'], p['bench'])
    cases = [
        ('mag',     (0, 1.6, 0),  'frame', 'magazine shoved up into the frame'),
        ('slide',   (0, -0.55, 0),'frame', 'slide pressed down through the frame'),
        ('trigger', (0, 0.9, 0),  'frame', 'trigger pushed up into the frame'),
    ]
    out = []
    for pid, d, qid, note in cases:
        off = [int(round(v / VOX)) for v in d]
        n = 0
        for (i, j, k) in byId[pid]['vox']:
            if (i + off[0], j + off[1], k + off[2]) in byId[qid]['world']:
                n += 1
        out.append((pid, qid, n, len(byId[pid]['vox']), note))
    return out


def check_staged(parts):
    """Parts that nest in the assembled model legitimately overlap; the staged
    view must not bury anything deeper than the assembled state does."""
    def offs(p, staged):
        bench = [int(round(v / VOX)) for v in GROUP_BENCH[p['group']]]
        extra = [int(round(v / VOX)) for v in GROUP_EXTRA.get(p['group'], [0, 0, 0])]
        st = [int(round(v / VOX)) for v in (p['staged'] if staged else [0, 0, 0])]
        return (bench[0]+extra[0]+st[0], bench[1]+extra[1]+st[1], bench[2]+extra[2]+st[2])

    bad = []
    ids = [p['id'] for p in parts]
    for i in range(len(ids)):
        for j in range(i+1, len(ids)):
            a, b = parts_by_id[ids[i]], parts_by_id[ids[j]]
            seat = overlap_count(shift(a['vox'], offs(a, False)), (0, 0, 0), shift(b['vox'], offs(b, False)))
            stg = overlap_count(shift(a['vox'], offs(a, True)), (0, 0, 0), shift(b['vox'], offs(b, True)))
            if stg > seat + CONTACT:
                bad.append((LABEL[a['id']], LABEL[b['id']], seat, stg))
    return bad


def main():
    global parts_by_id
    parts = load_all()
    parts_by_id = {p['id']: p for p in parts}
    print('--- calibration: deliberately wrong placements ---')
    for pid, qid, n, tot, note in calibrate(parts):
        print('  %-9s x %-9s %6d / %6d vox (%3d%%)  %s' % (pid, qid, n, tot, 100 * n // tot, note))
    print()
    report = plan(parts)

    print('%-11s %-36s %s' % ('part', 'step', 'result'))
    blocked = 0
    for pid, label, res, dist in report:
        if not res.startswith('clear'):
            blocked += 1
        print('%-11s %-36s %-46s %5.2f in' % (pid, label, res, dist))
    print('\n%d/%d paths verified collision-free' % (len(report) - blocked, len(report)))
    if blocked:
        print('press fits (need the final 2-5 mm of travel, model has no flex):')
        for pid, label, res, dist in report:
            if not res.startswith('clear'):
                print('   %s - %s' % (label, res))

    clash = check_staged(parts)
    if clash:
        print('staged layout overlaps: %d pair(s)' % len(clash))
        for a, b, seat, stg in clash:
            print('   %s / %s  assembled %d -> staged %d vox' % (a, b, seat, stg))
    else:
        print('staged layout: nothing buried deeper than in the assembled model')
    print()

    step_of = {}
    for i, (label, ids) in enumerate(STEPS):
        for pid in ids:
            step_of[pid] = i
    install_idx = {g: next(i for i, (l, ids) in enumerate(STEPS) if l == v)
                   for g, v in GROUP_INSTALL_STEP.items()}

    out = []
    for pt in parts:
        out.append(dict(id=pt['id'], label=pt['label'], cat=pt['cat'], group=pt['group'],
                        step=step_of.get(pt['id'], -1),
                        staged=[round(v, 3) for v in pt['staged']],
                        min=[round(v, 4) for v in pt['min']],
                        max=[round(v, 4) for v in pt['max']],
                        tris=pt['tris'],
                        data=base64.b64encode(pt['data']).decode('ascii')))

    press_ids = [pid for pid, label, res, dist in report if not res.startswith('clear')]
    press = sorted({step_of[x] for x in press_ids})

    with open(os.path.join(HERE, 'build_report.txt'), 'w') as fh:
        fh.write('Glock 17 assembly plan\nvoxel %.3f in, contact tolerance %d\n\n'
                 % (VOX, CONTACT))
        for pid, label, res, dist in report:
            fh.write('%-36s %-52s standoff %.2f in\n' % (label, res, dist))
        fh.write('\n%d/%d verified clear\n' % (len(report) - blocked, len(report)))
    js = 'window.PLAN=' + json.dumps({
        'vox': VOX,
        'steps': [x[0] for x in STEPS],
        'groups': {g: {'bench': list(GROUP_BENCH[g]),
                       'extra': GROUP_EXTRA.get(g, [0, 0, 0]),
                       'dir': list(GROUP_DIR.get(g, (0, 0, 0))),
                       'install': install_idx.get(g, -1)}
                   for g in GROUP_BENCH},
        'press': press,
    }, separators=(',', ':')) + ';\n'
    js += 'window.PARTS=' + json.dumps(out, separators=(',', ':')) + ';\n'
    open(os.path.join(HERE, 'parts.js'), 'w').write(js)
    print('\nwrote parts.js (%.2f MB)' % (len(js) / 1e6))
    return 1 if blocked else 0


if __name__ == '__main__':
    sys.exit(main())
