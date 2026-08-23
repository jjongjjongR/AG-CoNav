#!/usr/bin/env python3
"""그림: 이기종 3로봇이 한 시뮬레이션에 함께 올라간 장면.

`capture_gz_shots.sh` 가 받은 원본 스크린샷에 로봇 이름표만 얹는다.
원본은 results/final_maps/gz_three_robots_raw.png.

라벨 좌표는 캡처 시점(gz_camera_shots.py 의 VIEWS[0])에 고정된 값이다.
카메라를 바꾸면 여기 픽셀 좌표도 다시 잡아야 한다.
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(WS, 'results/final_maps')
SRC = os.path.join(OUT, 'gz_three_robots_raw.png')

# (로봇 픽셀 위치, 라벨 위치, 라벨) — VIEWS[0] 시점 기준
MARKS = [
    ((600, 558), (300, 380), '4족  Unitree Go2\nrl_quadruped_controller'),
    ((1120, 495), (1420, 250), '드론  Gazebo 멀티콥터\n하향 OS1-32, 탐지 고도 84 m'),
    ((1090, 725), (1450, 850), '4륜  Clearpath Husky A300'),
]


def main():
    img = plt.imread(SRC)
    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=(w / 150, h / 150), dpi=150)
    ax.imshow(img)
    ax.set_xlim(0, w); ax.set_ylim(h, 0); ax.axis('off')
    for (px, py), (lx, ly), text in MARKS:
        ax.annotate(
            text, xy=(px, py), xytext=(lx, ly),
            fontsize=12, color='#101010', ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.45', fc='white', ec='#444444',
                      alpha=0.92, lw=1.0),
            arrowprops=dict(arrowstyle='-|>', color='#cc2200', lw=1.8,
                            shrinkA=6, shrinkB=8))
    ax.text(0.008, 0.985,
            '서울 성동구 재현 월드 (577.9 × 481.8 m) — 이기종 3로봇 동시 구동',
            transform=ax.transAxes, ha='left', va='top', fontsize=13,
            bbox=dict(boxstyle='round,pad=0.4', fc='#eaf4ff', ec='#9dbfe0',
                      alpha=0.95))
    fig.subplots_adjust(0, 0, 1, 1)
    p = os.path.join(OUT, 'gz_three_robots.png')
    fig.savefig(p, dpi=150, bbox_inches='tight', pad_inches=0)
    print('saved', p)


if __name__ == '__main__':
    main()
