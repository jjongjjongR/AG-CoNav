#!/usr/bin/env python3
"""
sync_poses_to_world.py

실행 중인 gz sim world의 각 모델 현재 pose를 조회해서,
world 파일 안 해당 <include><name>...</name> 블록의 <pose> 줄만
정확히 교체합니다. 나머지 포맷(주석, 들여쓰기, 다른 블록)은 건드리지 않습니다.

사용법:
    python3 sync_poses_to_world.py Seongdong_gu.world

주의:
    - gz sim이 이미 실행 중이어야 합니다 (다른 터미널에서 gz sim ... 실행 상태).
    - 실행 전 world 파일을 자동으로 .bak으로 백업합니다.
    - 반드시 diff로 변경 내용을 확인한 뒤 사용하세요.
"""

import re
import subprocess
import sys
import shutil
from pathlib import Path


def get_model_list():
    """
    gz model --list 출력 예시:

        Requesting state for world [Seongdong_gu]...
        Available models:
            - Seongdong_gu
            - oak_tree_0
            - oak_tree_1
            ...
            - bump_010
            - ramp_025

    "    - 이름" 형태의 줄만 골라 이름을 추출.
    지형 모델 자체(world 이름과 동일한 항목)는 제외합니다.
    """
    result = subprocess.run(
        ["gz", "model", "--list"],
        capture_output=True, text=True, check=True
    )

    world_name_match = re.search(r"world \[([^\]]+)\]", result.stdout)
    world_name = world_name_match.group(1) if world_name_match else None

    names = []
    for line in result.stdout.splitlines():
        m = re.match(r"^\s*-\s*(\S+)\s*$", line)
        if not m:
            continue
        name = m.group(1)
        if name == world_name:
            # 지형(terrain) 모델 자체는 제외 - 보통 <include> 방식이 달라
            # 이름 기반 <pose> 치환 대상이 아님
            continue
        names.append(name)
    return names


def get_model_pose(name):
    """
    gz model -m <name> -p 출력 예시:

        Requesting state for world [Seongdong_gu]...
        Model: [12]
          - Name: oak_tree_0
          - Pose [ XYZ (m) ] [ RPY (rad) ]:
            [-175.123000 338.800000 0.000000]
            [0.000000 -0.000000 0.000000]

    "Pose [ XYZ ... ]:" 다음에 나오는 두 개의 대괄호 블록만 정확히 찾아서
    XYZ, RPY 값을 추출합니다. (Model: [12] 같은 다른 숫자는 무시)
    """
    result = subprocess.run(
        ["gz", "model", "-m", name, "-p"],
        capture_output=True, text=True, check=True
    )
    match = re.search(
        r"Pose\s*\[\s*XYZ.*?\]\s*\[\s*RPY.*?\]:\s*"
        r"\[([^\]]+)\]\s*\[([^\]]+)\]",
        result.stdout, re.DOTALL
    )
    if not match:
        raise ValueError(
            f"[{name}] pose 파싱 실패. 원본 출력:\n{result.stdout}\n"
            f"-> 이 출력 포맷을 대화창에 붙여넣어 주세요, 파싱 정규식을 맞춰드릴게요."
        )
    x, y, z = match.group(1).split()
    roll, pitch, yaw = match.group(2).split()
    return f"{x} {y} {z} {roll} {pitch} {yaw}"


def update_world_file(world_path: Path, poses: dict):
    content = world_path.read_text(encoding="utf-8")

    # <include> ... <name>모델이름</name> ... <pose>...</pose> ... </include>
    # 블록을 찾아서 그 안의 <pose> 줄만 교체.
    pattern = re.compile(
        r"(<include>(?:(?!</include>).)*?<name>\s*{name}\s*</name>(?:(?!</include>).)*?<pose[^>]*>)"
        r".*?"
        r"(</pose>)",
        re.DOTALL,
    )

    updated_names = []
    missing_names = []

    for name, pose_str in poses.items():
        block_pattern = re.compile(
            r"(<include>(?:(?!</include>).)*?<name>\s*" + re.escape(name) +
            r"\s*</name>(?:(?!</include>).)*?<pose[^>]*>)(?:(?!</pose>).)*?(</pose>)",
            re.DOTALL,
        )
        new_content, n = block_pattern.subn(
            lambda m: m.group(1) + pose_str + m.group(2), content
        )
        if n == 0:
            missing_names.append(name)
        else:
            content = new_content
            updated_names.append(name)

    world_path.write_text(content, encoding="utf-8")
    return updated_names, missing_names


def main():
    if len(sys.argv) != 2:
        print("사용법: python3 sync_poses_to_world.py <world파일경로>")
        sys.exit(1)

    world_path = Path(sys.argv[1])
    if not world_path.exists():
        print(f"파일을 찾을 수 없습니다: {world_path}")
        sys.exit(1)

    backup_path = world_path.with_suffix(world_path.suffix + ".bak")
    shutil.copy2(world_path, backup_path)
    print(f"백업 완료: {backup_path}")

    print("모델 목록 조회 중...")
    names = get_model_list()
    print(f"{len(names)}개 모델 발견: {names}")

    poses = {}
    for name in names:
        try:
            poses[name] = get_model_pose(name)
            print(f"  {name}: {poses[name]}")
        except Exception as e:
            print(f"  [경고] {name} pose 조회 실패: {e}")

    updated, missing = update_world_file(world_path, poses)

    print("\n=== 결과 ===")
    print(f"업데이트된 모델 ({len(updated)}개): {updated}")
    if missing:
        print(f"world 파일에서 매칭 안 된 모델 ({len(missing)}개): {missing}")
        print("-> 이 모델들은 <name> 태그가 없거나 이름이 다를 수 있습니다. 수동 확인 필요.")

    print(f"\n변경 확인: diff {backup_path} {world_path}")


if __name__ == "__main__":
    main()
