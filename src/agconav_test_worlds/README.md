# agconav_test_worlds

Small worlds carved out of the full AG-CoNav worlds, for experiments where
loading the entire Seongdong-gu terrain is wasted time — bag recording, SLAM
runs (GLIM), quick physics checks.

**Only the map and the scan's pose range are specific to a test world.** Every
flight module is agconav_drone's, unmodified — `drone_path_player`,
`drone_pose_controller` and their yaml — so altitude, cruise speed, strip
spacing and turn behaviour are identical to a full-world run and the same setup
points back at the full world unchanged.

## Seongdong_gu_100x100

A 100 x 100 m block taken from `agconav_worlds`' `Seongdong_gu.world`.

| | |
|---|---|
| box | x `[-38.6, 61.4]`, y `[-166.1, -66.1]` (centre `11.4, -116.1`) |
| terrain | 257 x 257 heightmap (0.391 m/px), elevation 1.202 .. 6.452 m, relief 5.25 m |
| buildings | 20, 288 triangles, tallest 8.8 m — all fully inside the box |
| dropped | 8 buildings that straddled the boundary, 77 out-of-box includes |
| spawn clearing | `(-8.60, -86.10)`, 13.5 m from the nearest building, local std 0.076 m |

Terrain was verified in-sim against the source heightmap at 10 points: all
within **0.01 m**.

Robot spawns keep the relative layout and ground clearances they have in
`agconav_sim.launch.py`; only the clearing moved. The generator prints the
arguments to pass:

```
wheel_x:=-8.6000 wheel_y:=-86.1000 wheel_z:=2.2363 wheel_yaw:=-0.4349
leg_x:=-7.7890  leg_y:=-82.8070  leg_z:=2.3025  leg_yaw:=-0.4613
```

The drone (`X3`) is relocated inside the world file itself, to `(-5.398,
-85.458, 2.150)`. The `<world name>` is still `Seongdong_gu`, so
`world:=Seongdong_gu` keeps working.

### Running the scan

```bash
ros2 launch agconav_test_worlds scan.launch.py
ros2 launch agconav_test_worlds scan.launch.py record:=false headless:=true
```

Records `/drone/points`, `/drone/imu`, `/tf`, `/tf_static` and `/ground_truth/tf`
to `bags/drone_scan_100x100`. Note this is the *input* a SLAM system needs — raw
scans — not the elevation-map output the existing `maps/` bags hold.

### Regenerating

```bash
python3 scripts/generate_test_world.py --center 11.4 -116.1 --size 100 \
        --output worlds/Seongdong_gu_100x100
python3 scripts/generate_scan_path.py       # waypoints for whatever world it points at
```

`generate_scan_path.py` is a wrapper: it loads agconav_drone's
`generate_path.py` and only redirects its input world and output file, so the
flight parameters live in exactly one place.

## Things worth knowing before editing the generator

**Heightmap normalisation.** gz-common's `ImageHeightmap` scales terrain by the
*brightest pixel present in the image*, not by the full 16-bit range. Copying a
crop's pixels through unchanged therefore inflates it: a 4.2 .. 7.0 m patch came
out at 11.97 .. 18.40 m, because the crop's peak pixel was 28493 where the
source's is 65474. The generator re-stretches the crop over the full 16-bit
range and writes `size.z`/`pos.z` to match its true span and floor.

**Sampling resolution.** The heightmap side is chosen so a crop is never sampled
coarser than the source (0.701 m/px). A 120 m box at 129 px would be 0.93 m/px,
which flattens peaks by over a metre.

**Buildings are kept or dropped whole.** A sliced building is a hollow shell the
LiDAR sees through. `--buildings inside` (default) keeps only those fully within
the box; `--buildings touching` keeps any that reach in, which is how a 50 m
world ended up dominated by an 80 m tower whose footprint hung far past the
terrain edge.

**Verifying terrain by dropping things.** Don't. A sphere released from 10 m
tunnels straight through the DART heightfield — in the *original* world too, at
the very spots the robots stand on. Every "landing" in such a test is a building
roof. Set a 0.6 m cube 0.1 m above its expected height instead and check it
stays there. And build the probe world **inside the world's own directory**: a
probe written to a scratch dir resolves `mesh/height_map.png` against whatever
happens to be there, which silently measured a stale world through two rounds of
false diagnosis.
