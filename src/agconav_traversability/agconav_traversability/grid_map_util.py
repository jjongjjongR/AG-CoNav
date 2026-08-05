"""GridMap ↔ numpy 변환 (모듈 F 내부 전용).

grid_map_core 규약:
- 레이어 하나는 (size_x, size_y) 크기의 Eigen 행렬이고, Float32MultiArray에
  column-major(열 우선)로 담겨 온다. dim[0]=column_index(=size_y),
  dim[1]=row_index(=size_x).
- 인덱스 (i, j)의 시작점은 지도의 좌상단(= 최대 x, 최대 y)이고, i는 -x 방향,
  j는 -y 방향으로 증가한다.
- 지도가 이동하면 데이터를 다시 쓰지 않으려고 순환 버퍼로 저장하므로
  start_index만큼 밀려 있을 수 있다. 읽을 때 되돌린다.
- 미관측 셀은 NaN (README §3.3).
"""
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def layer_to_array(grid_map, layer):
    """GridMap의 레이어 하나를 (size_x, size_y) 배열로 꺼낸다."""
    data = grid_map.data[grid_map.layers.index(layer)]
    size_y = data.layout.dim[0].size
    size_x = data.layout.dim[1].size
    # column-major 데이터를 (size_y, size_x)로 읽고 전치하면 (size_x, size_y)가 된다.
    array = np.asarray(data.data, dtype=np.float32).reshape(size_y, size_x).T
    # 순환 버퍼를 원래 인덱스 순서로 되돌린다.
    array = np.roll(array, -grid_map.outer_start_index, axis=0)
    return np.roll(array, -grid_map.inner_start_index, axis=1)


def array_to_layer(array):
    """(size_x, size_y) 배열을 GridMap 레이어(Float32MultiArray)로 만든다."""
    size_x, size_y = array.shape
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(label='column_index', size=size_y,
                            stride=size_x * size_y),
        MultiArrayDimension(label='row_index', size=size_x, stride=size_x),
    ]
    # column-major로 펴려면 전치 후 행 우선으로 편다.
    message.data = array.T.reshape(-1).astype(np.float32).tolist()
    return message


def to_occupancy_grid(values, info, header):
    """(size_x, size_y) 점유값 배열을 OccupancyGrid로 만든다.

    GridMap은 좌상단(최대 x·y)부터 -x, -y로 훑고 OccupancyGrid는 좌하단
    원점에서 +x가 먼저 증가하는 row-major라, 두 축을 뒤집고 전치한다.
    지도 회전은 없다고 본다(README §3.1: 전역 프레임 map 하나).
    """
    size_x, size_y = values.shape
    grid = OccupancyGrid()
    grid.header = header
    grid.info.resolution = info.resolution
    grid.info.width = size_x
    grid.info.height = size_y
    grid.info.origin.position.x = info.pose.position.x - info.length_x / 2.0
    grid.info.origin.position.y = info.pose.position.y - info.length_y / 2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = values[::-1, ::-1].T.reshape(-1).tolist()
    return grid
