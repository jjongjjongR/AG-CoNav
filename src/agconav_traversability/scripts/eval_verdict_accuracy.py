#!/usr/bin/env python3
"""저장된 2.5D 고도 지도의 주행성 판정을 정답 지형과 대조하는 공용 도구.

    eval_verdict_accuracy.py <maps/TAG/drone_elevation_map> [--robot wheel]

`eval_verdict_aligned.py` 와 `eval_connectivity.py` 가 여기서
`LEG_STEP`, `WHEEL_STEP`, `load_map`, `max_step` 을 가져다 쓴다.

!! 이 파일은 복원본이다 !!
커밋 02926e2(실험 인프라 정리) 때 유실됐다. 저장소·git 이력·디스크 어디에도
남아 있지 않은데 `eval_connectivity.py` 가 계속 import 하고 있어서 그대로는
실행되지 않았다. 아래 세 가지는 전부 **운용 코드에서 그대로 가져온 정의**라
재구현이 아니라 복원이다.

  단차 문턱   `agconav_traversability/config/traversability_{wheel,leg}.yaml`
              의 `max_step_m` (wheel 0.08 / leg 0.15)
  단차 계산   `terrain_feature_calculator.py` 의 `compute_features` 와 동일
              (인접 8셀과의 최대 높이차)
  지도 읽기   `replay_elevation_map.py` 의 mcap 읽기 +
              `grid_map_util.py` 의 `layer_to_array` 규약
"""
import argparse
import json
import os

import numpy as np

# README §3.3 통과 기준. 운용 판정 노드가 쓰는 값과 같아야 한다.
WHEEL_STEP = 0.08
LEG_STEP = 0.15


def _shift(array, di, dj):
    """array 를 (di, dj) 만큼 민다. 밀려 나간 자리는 NaN."""
    ny, nx = array.shape
    out = np.full_like(array, np.nan)
    out[max(0, -di):ny - max(0, di), max(0, -dj):nx - max(0, dj)] = \
        array[max(0, di):ny - max(0, -di), max(0, dj):nx - max(0, -dj)]
    return out


def max_step(elevation, valid):
    """인접 8셀과의 최대 높이차와, 그 값이 유효한지를 돌려준다.

    `terrain_feature_calculator.compute_features` 의 단차 계산과 같다.
    이웃이 지도 밖이거나 미관측이면 그 방향은 빠지고 남은 이웃으로 계산한다
    (np.fmax 는 NaN 을 무시한다).

    valid 는 elevation 의 각 칸이 관측된 칸인지를 나타낸다. 관측 안 된 칸의
    값은 호출부에서 0 으로 채워 넘기므로, 여기서 NaN 으로 바꿔 계산에서
    빼야 한다 — 안 그러면 미관측 0 과의 차이가 거대한 단차로 잡힌다.
    """
    z = np.where(valid, elevation.astype(np.float64), np.nan)
    step = np.full(z.shape, np.nan)
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            step = np.fmax(step, np.abs(_shift(z, di, dj) - z))
    have = np.isfinite(step)
    return np.where(have, step, 0.0), have


def load_map(uri):
    """저장된 고도 지도(mcap)를 (z, xs, ys, res) 로 읽는다.

    z 는 (len(ys), len(xs)) 이고 xs·ys 는 **오름차순**이다. 미관측 칸은 NaN.

    !! GridMap 의 인덱스 규약을 그대로 따라야 한다 !!
    grid_map 은 좌상단(최대 x·y)에서 시작해 -x, -y 방향으로 훑고, 데이터는
    column-major 에 순환 버퍼(outer/inner_start_index)다. 그래서
      (1) column-major 로 읽어 전치하고, (2) 순환 버퍼를 되돌리고,
      (3) 두 축을 뒤집어야 오름차순 좌표계가 된다.
    이 순서를 하나라도 빠뜨리면 지도가 뒤집히거나 밀린 채로 정답과 비교돼
    FN 이 통째로 잘못 나온다.
    """
    from grid_map_msgs.msg import GridMap
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id='mcap'),
                ConverterOptions('', ''))
    grid = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic.endswith('elevation_map'):
            grid = deserialize_message(data, GridMap)
    if grid is None:
        raise SystemExit('%s 에 elevation_map 메시지가 없다.' % uri)

    layer = 'elevation'
    data = grid.data[grid.layers.index(layer)]
    size_y = data.layout.dim[0].size
    size_x = data.layout.dim[1].size
    # column-major -> (size_x, size_y)
    arr = np.asarray(data.data, dtype=np.float64).reshape(size_y, size_x).T
    # 순환 버퍼를 원래 인덱스 순서로
    arr = np.roll(arr, -grid.outer_start_index, axis=0)
    arr = np.roll(arr, -grid.inner_start_index, axis=1)

    res = grid.info.resolution
    cx = grid.info.pose.position.x
    cy = grid.info.pose.position.y
    # arr[i, j] 는 x 가 큰 쪽 -> 작은 쪽, y 가 큰 쪽 -> 작은 쪽 순서다.
    # 두 축을 뒤집고 전치해 (ny, nx) 오름차순으로 만든다.
    z = arr[::-1, ::-1].T
    xs = cx - grid.info.length_x / 2.0 + (np.arange(size_x) + 0.5) * res
    ys = cy - grid.info.length_y / 2.0 + (np.arange(size_y) + 0.5) * res
    return z, xs, ys, res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--robot', default='wheel', choices=('wheel', 'leg'))
    a = ap.parse_args()
    z, xs, ys, res = load_map(a.map)
    ok = np.isfinite(z)
    step, have = max_step(np.where(ok, z, 0.0), ok)
    th = WHEEL_STEP if a.robot == 'wheel' else LEG_STEP
    print(json.dumps({
        'map': os.path.basename(a.map.rstrip('/')),
        'cells': int(ok.sum()),
        'resolution': float(res),
        'x_range': [float(xs[0]), float(xs[-1])],
        'y_range': [float(ys[0]), float(ys[-1])],
        'pass_pct': float(100.0 * ((step <= th) & have & ok).sum()
                          / max(1, int((have & ok).sum()))),
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
