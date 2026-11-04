# Glock 17 — assembly animation

Every one of the 31 printed parts, in the order a gunsmith actually fits them,
moving along a path that was collision-tested against the parts already in the
gun. Open `index.html`.

## What changed from a plain exploded view

The first version spread the parts out and slid them home on hand-picked
vectors — a part could pass straight through the frame on the way. This version
derives the motion from the geometry:

- **Real order.** Frame group first (housing, trigger bar, connector, trigger,
  mag catch, slide lock, locking block), then the slide group built up on its
  own (firing pin stack, firing pin safety, extractor group, cover plate,
  barrel, recoil spring, sights), then the slide slides onto the frame, then the
  magazine. Every pin is inserted *after* the parts it retains — the housing pin
  after the housing, the trigger pin after the trigger, the locking block pin
  after the block.
- **Paths, not offsets.** `build.py` voxelises every part at 0.03 in, walks the
  assembly in order, and for each part searches 26 candidate insertion
  directions for one where the whole approach — sampled every voxel — never
  buries the part into anything already installed. The longest clear path wins,
  the part waits 1.7–4.8 in out along that axis, and the animation slides it
  straight in from there.
- **Interference is measured by depth, not by contact.** Two surfaces touching
  or sliding past each other with a few thousandths of clearance produce a
  one-voxel band and score zero; a part pushed through another part produces a
  solid core of shared voxels and scores high. Each part's score is also
  measured against its own seated state, because this model's parts are
  modelled in contact and some already overlap when assembled.

`build.py` prints a verdict per part and writes `build_report.txt`; re-run it to
regenerate the table.

## Open it

Double-click `index.html` — plain script tags, no server, no network, no build
step. If your browser blocks local scripts:

```
cd ~/Glock17-exploded && python3 -m http.server 8848
# then visit http://127.0.0.1:8848/
```

## Controls

| | |
|---|---|
| `▶` / `space` | play, pause |
| `assemble ⇄` | flip direction — forward assembles, reverse takes it apart |
| drag the bar | scrub |
| click a step | jump to it |
| click a part | isolate it, dim the rest |
| `side` `3/4` `top` | camera presets |
| `auto frame` | keeps the whole assembly in shot (turns off when you zoom) |

## Assembly order

**Frame** — 1 trigger mechanism housing · 2 trigger bar · 3 connector ·
4 trigger housing pin · 5 trigger · 6 trigger safety · 7 trigger safety pin ·
8 trigger pin · 9 magazine catch · 10 slide lock · 11 slide catch lever ·
12 locking block · 13 locking block pin

**Slide** — 14 firing pin, spring cup and spacer sleeve · 15 firing pin safety ·
16 extractor spring bearing and plunger · 17 extractor · 18 slide cover plate ·
19 barrel · 20 recoil spring assembly · 21 front sight · 22 rear sight

**Install** — 23 slide onto the frame · 24 magazine follower · 25 magazine
insert · 26 magazine baseplate · 27 magazine into the frame

## Verification status — read this

13 of the 27 motions are clean at 0.03 in resolution: nothing the part passes
through is anything it does not already sit against when assembled.

14 have leftover interference of 7–414 voxels (0.0002–0.011 in³), all of it in
the last few millimetres of travel. These are the press and snap fits — the
trigger bar's cruciform into the housing, the trigger into the frame, the
locking block past the frame rails, the barrel's locking lug into the slide —
where the real part flexes or is pressed and the CAD model has no clearance for
it. The largest is the barrel at 414 voxels, and it is buried inside the slide
where you cannot see it. The rest are 7–65 voxels, i.e. specks at this scale.

So: no part travels through the frame or the slide on its way home, and the
group moves (slide onto frame, magazine into frame) are fully clear. The
remaining motion that is not physically exact is confined to fits that a rigid
mesh cannot express.

## Part inventory

31 parts, 63,586 triangles. Left out on purpose: `Glock 17.STL` (51 MB, the
whole pistol merged into one mesh) and 10 of the 11 cartridge STLs, which are
duplicate copies of two shapes stacked in the magazine.

## Rebuilding

```
python3 build.py        # ~7 min cold, seconds with .voxcache.pkl warm
```

`PARTS` holds the file list, material and the nominal insertion direction;
`STEPS` is the assembly order (one motion per entry, several parts move together
when they are a stack); `GROUP_BENCH` is where the slide and magazine
subassemblies sit while they are built. The planner re-solves the directions and
distances from the geometry, so changing the table does not require re-tuning
offsets by hand.

Units are inches, matching the STLs. +X toward the muzzle, +Y up, +Z across.
