#!/usr/bin/env bash
# Gazebo 월드 로드 완료를 기다린다. 로드되면 0, 시간 초과면 1로 종료한다.
#
# 성동구 월드는 heightmap(717x665 m)과 건물 메시(763 KB)가 커서 로드에 시간이
# 걸린다. 그 전에 ros_gz_sim의 create 노드가 spawn을 시작하면
# "Requesting list of world names"만 반복하며 멈춘다(약 1/5 확률로 재현).
# 이 스크립트로 월드가 실제로 응답할 때까지 막아 두고 spawn을 시작한다.
#
# 사용법: wait_for_world.sh <월드이름> [타임아웃초]
set -u

WORLD="${1:?월드 이름이 필요합니다}"
TIMEOUT="${2:-180}"
START=$(date +%s)
DEADLINE=$(( START + TIMEOUT ))

echo "[wait_for_world] '${WORLD}' 로드 대기 (최대 ${TIMEOUT}s)"

while true; do
  # 서비스 목록에 있는 것만으로는 부족하다. 실제로 호출해 월드 이름이
  # 돌아와야 서버가 응답 가능한 상태다.
  if gz service -s /gazebo/worlds \
       --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V \
       --timeout 2000 -r 2>/dev/null | grep -q "\"${WORLD}\""; then
    # create 서비스까지 올라와야 spawn이 성공한다.
    if gz service -l 2>/dev/null | grep -qx "/world/${WORLD}/create"; then
      echo "[wait_for_world] '${WORLD}' 로드 완료 — spawn 시작"
      exit 0
    fi
  fi

  # 오래 걸리면 gz 프로세스 상태를 남긴다. 이 실패는 재현이 드물어서
  # 사후에 원인을 못 찾는다. 살아는 있는지(state), 무엇을 기다리는지(wchan),
  # 자식 프로세스(server/gui)가 떴는지를 그 순간에 찍어둔다.
  NOW=$(date +%s)
  if [ $(( (NOW - START) % 30 )) -eq 0 ] && [ "${NOW}" -ne "${LAST_DIAG:-0}" ]; then
    LAST_DIAG="${NOW}"
    echo "[wait_for_world] $((NOW - START))s 경과 — gz 프로세스 상태:"
    pgrep -af "gz sim" || echo "  (gz sim 프로세스 없음)"
    for p in $(pgrep -f "gz sim" 2>/dev/null); do
      echo "  pid=${p} state=$(awk '{print $3}' "/proc/${p}/stat" 2>/dev/null)" \
           "wchan=$(cat "/proc/${p}/wchan" 2>/dev/null)" \
           "threads=$(awk '/Threads/{print $2}' "/proc/${p}/status" 2>/dev/null)"
    done
  fi

  if [ "${NOW}" -ge "${DEADLINE}" ]; then
    echo "[wait_for_world] 타임아웃 ${TIMEOUT}s — 월드가 뜨지 않았습니다" >&2
    echo "[wait_for_world] 마지막 gz 상태:" >&2
    pgrep -af "gz sim" >&2 || echo "  (gz sim 프로세스 없음)" >&2
    exit 1
  fi
  sleep 1
done
