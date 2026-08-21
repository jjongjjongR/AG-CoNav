# PROGRESS — 5m AGL 방법B 경로 실험 (2~5단계)

## [새 세션] gicp-preprocessing 재개 — UTM 크래시 복구 확인 후 사용자 요청으로 즉시 재중단

**전체 타임아웃 3시간(재개 확인 포함) 중 약 20분만 사용.** 직전 세션이
UTM 크래시로 중단됐다는 전제로 재개 절차(-1단계)를 밟던 중, **재개
확인 도중 사용자가 "멈춰, 집 가서 다시 킬게"로 즉시 중단을 요청** —
그 시점에 진행 중이던 작업만 안전하게 정지하고 이 세션은 여기서
종료한다(추가 작업 없음).

### -1. 재개 지점 파악 결과
- `pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음, **gicp-gt-pose
  관련 프로세스도 없음**(안전 경계 위반 없음, 별도 조치 불필요).
- 현재 브랜치: `gicp-preprocessing`(이미 생성/체크아웃돼 있었음,
  origin과 동기화 상태).
- `git log` HEAD = `e2c3808`("SOR/평탄면필터 결과 확정, 조합은 진행
  중 (안전 체크포인트)") — 커밋 메시지 자체가 이미 "조합(B+C) 빌드는
  이 커밋 시점 기준 92% 진행 중"이라고 정확히 기록해둠.
- `gicp_preprocessing_result.md` 확인: 0단계(재비행 판단)~3단계(B/C
  개별 검증) 완료, **4단계(조합) 진행 중이던 것이 마지막 기록** —
  지시사항이 예상한 단계 구분과 정확히 일치.

### 손상/불완전 여부 확인 — combo(B+C) 조합의 중간 산출물만 유실
- `run_results/logs/preprocessing_experiment.log`(커밋에 없는
  unstaged 변경분) 확인: `BUILD start: combo`(16:19:42) →
  `BUILD done: combo (exit 0)`(17:49:25) → `SCORE start:
  methodb_combo`(17:49:25)에서 로그가 끊김 — **combo 빌드 자체는
  실제로 완주했고, 채점(scoring) 시작 직후 크래시**가 난 것으로 확인.
- 그러나 산출물이 있어야 할 `/tmp/gicp_preproc/`(baseline_combo.npy,
  methodb_combo.npy)가 **디렉토리째로 사라짐** — `uptime -s`/`who -b`
  로 부팅 시각이 17:57:01임을 확인, 로그 마지막 시각(17:49:25)보다
  나중이므로 **UTM 크래시가 VM 재부팅을 유발했고 `/tmp`(rootfs 위,
  휘발성 취급됨)가 재부팅으로 비워진 것**으로 결론. `find / -iname
  "*gicp_preproc*"`/`"*methodb_combo*"` 둘 다 전무 확인.
- **SOR(B)·평탄면(C) 개별 결과는 무사** — 이 두 변형의 수치는 이미
  `gicp_preprocessing_result.md`(git 커밋 `e2c3808`에 포함)에 직접
  기록돼 있어 `/tmp` 산출물이 사라져도 영향 없음(원본 JSON은 스크립트
  설계상 애초에 `/tmp`에만 있고 결과 문서에 값만 옮겨 적는 방식이라,
  "기록은 됐는데 결과 파일이 없는" 경우가 SOR/평탄면에는 해당하지
  않음 — 둘 다 스코어링까지 완주해서 그 수치가 문서에 그대로 옮겨져
  있음, `SCORE done: methodb_sor`/`methodb_flatness` 로그로도 재확인).
- 원본 bag(`bags/velocity_4m_5mps_preproc_src`, 27.1GiB, mcap 15개)은
  `ros2 bag info`로 정상 인식 확인 — 손상 없음, 재사용 가능.
  `build_methodB_cloud_preprocess.py`/`run_preprocessing_experiment.sh`
  둘 다 `py_compile`/육안 검토로 문법·로직 이상 없음 확인.
- **결론: 유일하게 미완료/유실된 것은 combo(B+C) 조합 하나뿐**이고,
  원인은 코드/데이터 손상이 아니라 `/tmp` 휘발성 때문 — 재실행만
  하면 된다(재설계 불필요).

### 디스크 확인
`df -h ~` → 78%(17G 여유). 임계값(70%) 초과 상태였으나, `bags/`에
`velocity_4m_5mps_preproc_src`(28G) 외 다른 bag이 없어(이미 이전
세션들이 정리 완료) 추가로 지울 스크래치가 없음 — combo 재실행에
필요한 `/tmp` 여유(baseline+methodb 합쳐 최대 약 8GB, float32×3열
기준 추정)는 17G 여유 안에서 충분히 감당 가능하다고 판단해 **별도
정리 없이 진행**하기로 함(애매하게 지우지 않는다는 원칙 적용 — 유일한
후보인 bag 자체는 이번 재실행에 필요하므로 지울 수 없음).

### combo 재실행 착수 — 시작 직후 사용자 요청으로 즉시 중단
`bash run_results/run_preprocessing_experiment.sh
bags/velocity_4m_5mps_preproc_src combo`를 백그라운드로 시작(이전과
동일한 커맨드, 이전 실적 기준 빌드 약 90분+채점 약 15분 예상). **시작
후 수 초 이내(빌드 스크립트가 bag을 열기 시작한 직후) 사용자가 "멈춰
일단 집가서 다시 킬게"로 중단을 요청** — 즉시 `TaskStop` +
`kill -TERM`(PID 3874 python 빌드 프로세스, 3852/3850 래퍼 bash)으로
정지, 2초 대기 후 `pgrep`로 관련 프로세스 완전히 사라졌음을 확인.
진행이 거의 없었던 시점(bag 열기 직후, GICP 처리 시작 전으로 추정)
이라 **중간 산출물이 사실상 없었음** — `/tmp/gicp_preproc/`를 안전하게
`rm -rf`로 정리(불완전한 재시작 흔적이 다음 세션에 "이미 진행 중"으로
오인되는 것을 방지). `pgrep -af "ros2|gz sim|docker"` 최종 확인 →
dockerd 외 잔여 프로세스 없음. `git status` → 커밋 안 된 변경은
`preprocessing_experiment.log`(BUILD start 로그 한 줄 추가된 것,
경미) 뿐 — 이 파일까지 포함해 이번 세션 진행 상황을 커밋한다.

### 다음 세션이 이어받을 때
1. **combo(B+C) 빌드+채점을 처음부터 다시**: `bash
   run_results/run_preprocessing_experiment.sh
   bags/velocity_4m_5mps_preproc_src combo` 그대로 재실행하면 됨(코드
   수정 불필요, 재설계 불필요) — 이전에 이 커맨드로 빌드까지는 정상
   완주가 실측 확인된 바 있음(로그 참고).
2. 완료되면 `gicp_preprocessing_result.md`의 `<!-- COMBO_* -->`
   플레이스홀더(2·4절)를 실제 수치로 채우고, 5절(결론)을 조합 결과
   반영해 갱신, 6절(저장/보고) 그대로 진행.
3. **교훈**: `/tmp`는 이 VM에서 재부팅 시 휘발된다 — 장시간(1시간+)
   걸리는 빌드의 중간/최종 산출물을 `/tmp`에만 두면 크래시 시 전부
   재계산해야 한다. 여유가 되면 스코어링 직전에 `.npy`를
   `run_results/` 등 영속 경로로 잠깐 복사해두는 안전장치를 고려할
   가치가 있음(이번 세션 범위 밖이라 적용하지 않음).

### 저장/보고
이 절 자체와 `preprocessing_experiment.log`(BUILD start: combo 로그
한 줄) 커밋. 새로 생성된 파일 없음(재실행이 시작 직후 중단돼 산출물
없음). push까지 완료.

## [새 세션] Phase 4 — glim_ext gnss_global(실시간 GPS 제약) 시도 — 결론: 미채택

**전체 타임아웃 2시간.** 목표: 사후보정이 아니라 GLIM 최적화 과정
자체에 GPS를 실시간 제약으로 넣는 `glim_ext`의 `libgnss_global.so`를
cv1e3(직전 세션이 확정한 최적 설정, 전체 bag ATE 68.6m)에 적용해
개선되는지 확인. **결과: 결론적으로 유효하지 않음 — 검증에 실패해
"미채택"으로 정리한다(단순 "미달"이 아니라 이 모듈 자체의 신뢰성
문제로 판단).**

### 1. gnss_global 입력 형식 조사
소스(`~/glim_ext_ws/src/glim_ext/modules/mapping/gnss_global/`) 직접
읽음 — NavSatFix를 직접 받지 않고 `geometry_msgs/PoseWithCovarianceStamped`
(고정 데카르트 프레임, position만 사용, orientation/covariance 무시)를
기대한다. `GlobalMappingCallbacks::on_insert_submap`/`on_smoother_update`
콜백으로 **submap 단위**(스캔 단위 아님)로 GTSAM
`PoseTranslationPrior` 팩터를 추가하는 방식 — "실시간 제약"은 맞지만
평활화 시점은 submap 생성 주기(`max_num_keyframes=50`≈5s)마다다.
자체 정렬(`T_world_utm`)은 **submap 궤적과 GPS 궤적을 SVD(Umeyama류,
XY만, Z는 평행이동으로만 처리)로 맞추는데, `min_baseline`(기본
10m) 조건을 만족하는 첫 순간 단 한 번만 계산하고 이후 다시 갱신하지
않는다** — 코드 주석 자체가 "very naive... ignores GNSS observation
covariance"라고 명시.

`run_results/diag_scripts/navsat_to_gnss_pose.py`(신규) 작성 —
`/drone/gps`(NavSatFix)를 `pymap3d.geodetic2enu`로 world 원점 기준
ENU 변환(3-2절에서 이미 검증된 기준점 재사용) 후 `/gnss`로 재발행.
`glim_config_cv1e3_gnss/`(cv1e3 사본)에 `libgnss_global.so` 등록 +
`config_gnss_global.json`(gnss_topic=/gnss, min_baseline=10.0,
prior_inf_scale=[1e3,1e3,0.0], 전부 glim_ext 기본값 그대로) 추가.

### 2. 40초 슬라이스 스모크 테스트 — 통과
모듈 로드/구독(publisher 1·subscriber 1 확인)/정상 종료 전부 이상
없음. 다만 이 짧은 창에서는 `T_world_utm=` 정렬 로그가 안 떴다(이후
알게 되지만, 정상 — 더 긴 시간/거리가 필요).

### 3. 전체 bag 1차 시도(12청크) — 세그폴트로 실패
청크 10까지 정상 진행, 20:36:23(재생 시작 34초 후)에
`T_world_utm=se3(-9.29,162.46,-11.12,...,yaw≈13.4°)` 정렬 로그 확인
— **GPS 제약이 실제로 걸리기 시작한 것 자체는 확인됨.** 그러나 청크
11(마지막, t≈2092~2254s 구간) 재생 중 t≈2135~2137s에서
`too few points in the downsampled cloud (0 points)` →
`warning: Empty point cloud` → **`Segmentation fault`**로 GLIM
프로세스 자체가 죽음(`/tmp/dump` 자체가 안 생겨 완전 손실).

원인 분석: 이 시점은 **직전 세션이 이미 확인해둔 "임무완료 후
자유낙하 구간"(`/drone/path_status=true` 발행 t≈2129.67s 직후)**과
정확히 일치 — 자유낙하로 라이다가 빈 반환을 주는 극단 케이스에서
GLIM이 안전하게 처리하지 못하고 죽는 것으로 보인다. 직전 세션의
cv1e3 단독 실행(gnss_global 없음)은 같은 구간을 무사히 통과했었는데,
이번엔 크래시 직전 `[mem] CPU memory usage: 4899.02/5894.34 MB
83.11%`로 우리 외부 메모리가드 임계값(4600MB)을 이미 넘어선 상태였다
(가드는 5초 주기 폴링이라 딱 그 사이에 못 잡음) — **gnss_global이
추가하는 submap/팩터 상태가 메모리를 유의미하게 더 쓰게 만들어(같은
지점에서 비-GNSS 버전 대비 RSS가 더 높았음), 이미 알려진 "빈
포인트클라우드" 버그가 메모리 압박과 겹쳐 세그폴트로 악화된 것으로
추정.**

### 4. 재시도(11청크로 축소) — 완주했으나 GPS 제약이 활성화 안 됨
크래시 구간(청크 11)을 아예 안 먹이도록 11청크(t≈9~2092s,
`ros2 bag info`로 청크별 경계 실측 확인)만 재생하도록 축소해 재실행 —
크래시 없이 정상 완주(`traj_lidar.txt` 8318 pose, t=[8.9,2097.2]s).

**그러나 로그 전체(26,783줄)를 검사한 결과 `T_world_utm=` 정렬 로그가
단 한 번도 안 떴다** — 즉 이번 실행에서는 GPS 제약이 끝까지
활성화되지 않았다. `/gnss` 컨버터는 정상 작동 확인(20,500건 이상
정상 재발행 로그 확인, GPS 데이터 자체는 확실히 흘러 들어감) —
그런데도 정렬이 안 걸렸다. **동일 설정, 거의 동일한 데이터(청크
0~10은 1차 시도와 완전히 동일)인데 1차 시도는 34초 만에 정렬이 됐고
이번엔 35분 내내 안 됐다** — 원인을 명확히 특정하지 못함(시간 예산
부족으로 `on_insert_submap` 콜백 자체가 호출됐는지까지는 추가
계측 없이 확인 불가). glim_ext README의 자체 경고("half-baked code
that may not be well-maintained")와 부합하는, **이 실험적 모듈의
활성화 자체가 실행마다 일관되지 않다는 신뢰성 문제**로 잠정 결론.

ATE 계산 결과(GPS 제약 없이 사실상 cv1e3와 동일 파이프라인이 11/12
청크만 처리된 것): 평균 81.8m, 중앙값 80.5m, 최대 160.8m —
cv1e3 단독(68.6m, 12청크 전체)보다 오히려 나쁘다. **이 숫자는 GPS
제약의 효과를 보여주는 게 아니다**(제약이 안 걸렸으므로) — 궤적
그림(`run_results/glim_diag_dumps/fullrun_cv1e3_gnss/ate_gt_vs_glim.png`)도
cv1e3 단독 결과와 거의 같은 형태(스트립 박스 위아래로 뒤엉킨 궤적)를
보여 이 해석과 일관된다. 차이(68.6m→81.8m)는 청크 11 누락 +
CT-GICP 재생 시점의 미세한 비결정성 때문으로 보인다(GPS 제약과
무관).

### 5. 결론 및 권고
**glim_ext의 gnss_global은 이번 세션 범위에서 신뢰성 있게 검증하지
못했다** — 1차는 제약이 걸렸지만 무관한 GLIM 버그로 크래시했고,
2차는 크래시는 피했지만 제약 자체가 안 걸렸다. 시간 예산(2시간)이
소진돼 원인을 더 파거나 3차 시도를 하지 않고 여기서 정리한다.

**권고 — 다음 세션은 이 실시간 제약 방식을 더 파지 말고, 이미 설계·
구현까지 끝나 있는 사후결합(post-hoc, `run_results/glim_gps_correct.py`,
GLIM(+GPS 사후결합) 전체 파이프라인 실험 절 3-3)으로 가는 게 낫다.**
근거:
1. 사후결합은 GPS 앵커마다 **주기적으로** 재정렬(SE3 보간)하는
   방식이라, 이번에 의심되는 "정렬을 초반에 딱 한 번만 계산하고 다시
   안 고친다"는 gnss_global의 구조적 약점(추정: 초반 궤적이 왕복
   스트립 패턴상 거의 직선이라 회전 추정이 취약할 가능성)이 애초에
   생기지 않는다.
2. GLIM 프로세스 자체의 실행(궤적 생성)과 GPS 보정을 완전히 분리하는
   구조라, 이번에 겪은 "모듈 활성화가 실행마다 달라짐" 같은 재현성
   문제에서 자유롭다 — 이미 확보된 cv1e3 궤적(`traj_lidar.txt`,
   9011 pose, 전체 bag)에 바로 적용 가능하고 재실행(GLIM 재구동) 자체가
   필요 없다(수 분 내 결과 확인 가능, 이번처럼 30~40분 재실행
   사이클이 필요 없음).
3. 사용자가 예시로 든 다른 후보(`smoother_lag`, `max_num_keyframes`,
   디스큐)는 순수 LIO 파라미터 튜닝 축이라 GPS 정보를 전혀 안 쓴다 —
   목표(ATE 23m 근처)는 애초에 GPS 같은 절대 기준 없이 순수 LIO
   드리프트만으로 달성하기 어려운 수준일 가능성이 높고(84m 실험도
   원래 GPS 없이 23m를 낸 것이지만 비행이 훨씬 짧았다, 236s vs
   2254s), 사후결합이 더 직접적인 해법으로 판단.

### 세션 재개용 참고 (다음 세션이 이어받을 경우)
1. **`glim_config_cv1e3_gnss/`, `navsat_to_gnss_pose.py`는 보존하되
   당장 재사용 계획 없음** — gnss_global 재시도는 위 권고에 따라
   후순위. 필요해지면 `on_insert_submap`이 실제 호출되는지부터
   직접 계측(예: `libglim_callback_demo.so`를 같이 등록해 콜백
   발생 여부 확인)하는 게 다음 디버깅 시작점.
2. **다음 최우선 작업**: `run_results/glim_gps_correct.py`를
   `run_results/glim_diag_dumps/fullrun_cv1e3/traj_lidar.txt`
   (cv1e3 단독, 9011 pose, t=[8.9,2258.3]s, 이미 검증됨)에 바로
   적용. GPS 앵커는 `bags/velocity_4m_5mps_gps_attempt2`의
   `/drone/gps`를 그대로 쓰되, **t<=2129.67s로 절단**(자유낙하 구간
   제외, 이미 두 세션에 걸쳐 확인된 경계).
3. 보정된 궤적으로 다시 `ate_full.py`(이번 세션 신규, 재사용 가능)로
   ATE 재계산 → 84m 실험 수준(23m 근처) 도달 여부 확인.
4. 도달하면 `glim_gps_build_map.py`+`glim_gps_metrics.py`로 최종
   4개 지표(coverage/wheel FN%/leg FN%/비행시간) 산출 후 5m AGL
   실험 비교표에 추가.

### 저장/보고
`glim_config_cv1e3_gnss/`, `run_results/diag_scripts/{navsat_to_gnss_pose.py,
run_glim_chunked_gnss.sh}`, `run_results/glim_diag_dumps/fullrun_cv1e3_gnss/`,
`run_results/logs/{smoke_gnss_*,fullrun_cv1e3_gnss_*}` 전부 커밋 대상
(실패 원인 재현/디버깅 참고용으로 보존). 잔여 프로세스 없음, 최종
디스크 71% 확인.

## [새 세션] Phase 2 파라미터 튜닝 재개 — 컴퓨터 전원 강제종료 후 복구

**전체 타임아웃 2시간.** 직전 세션이 "VM 재부팅이 아니라 컴퓨터 전원
자체가 꺼졌다"는 상황에서 중단됨 — 파일 쓰기 도중 강제종료 가능성이
있어 손상 여부부터 꼼꼼히 확인 후 이어감. 판단 지점은 확인 없이
스스로 진행, 근거는 이 문서에 기록.

### -1. 재개 지점 파악 + 손상 여부 확인
- `pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음(깨끗, 예상대로).
- **`git status`가 대량의 untracked 파일을 보여줌** — 직전 세션이
  `constant_velocity_inf_scale` 스윕(정확히 이번 지시사항의 Phase 2
  작업)을 이미 상당히 진행했으나, **PROGRESS.md에 전혀 기록하지
  못한 채 전원이 꺼진 것으로 판단**(위 "[새 세션] GLIM odometry 발산
  원인 진단" 절이 마지막 기록인데 거기엔 이 스윕 얘기가 없음). 즉
  이번 재개는 "기록되지 않은 작업"을 파일 흔적만으로 복원하는 것부터
  시작해야 했다.
- **파일별 손상 검사** (mtime 순서로 재구성):
  - `glim_config_cv{1e-1,1e-2,1e2,1e3,1e4}/`(18:16~18:21 생성) — 6개
    설정 사본(`glim_config`의 `constant_velocity_inf_scale`만 변경),
    전부 정상 크기/내용, 손상 없음.
  - `run_results/glim_diag_dumps/sweep_{baseline_1e0,cv1e-1,cv1e-2,
    cv1e2,cv1e3,cv1e4}/`(각 `odom_imu.txt`/`odom_lidar.txt`/
    `gt_vs_glim.png`) — **전부 정상 완주 확인**: 각 로그
    (`run_results/logs/sweep_*_glim.log`)가 `[global] saved`로
    깨끗하게 끝남(정상 종료), PNG는 `file` 명령으로 유효한 PNG로
    확인, odom txt는 마지막 줄까지 8개 float 필드가 개행문자로
    끝나는 완전한 줄로 확인(끊긴 줄 없음). **손상/미완료 없음 —
    이 6개 스윕은 재실행 불필요.**
  - `run_results/logs/fullrun_cv1e3_glim.log`(112KB, 1230줄) —
    **이게 크래시 순간에 쓰이고 있던 파일**: 마지막 줄이
    `large time gap ... diff=0.600000`로 정상적인 GLIM 경고 로그
    형식이지만 뒤에 `[global] saved`나 종료 로그가 전혀 없이 뚝
    끊김(마지막 스캔 시각 t=577.8s, 시작 18:24:13~중단 18:34:14,
    약 10분 경과). `run_results/glim_diag_dumps/fullrun_cv1e3/`
    디렉토리는 **완전히 비어있음**(dump 자체가 저장되기 전에
    죽음 — SIGKILL/전원차단이라 정상 종료 훅이 못 불렸을 것).
    **이 파일이 만들던 작업 단계(3단계: cv1e3로 전체 재실행) 전체를
    "미완료"로 간주** — 지시사항 그대로 처음부터 다시 진행.
  - `bags/_chunk_param/_chunk_param_0.mcap`(1.4GB) — fullrun_cv1e3의
    청크 0을 t필드 추가 중 만들던 중간 산출물, 전원 차단 시점에
    쓰다 만 것(파일 자체는 존재하지만 원본 GPS bag에서 언제든
    재생성 가능한 스크래치 파일). **삭제**(디스크 정리 겸, 아래
    0-1절).
  - `bags/_slice40`(453MB, `make_slice_with_t.py`로 만든 40초 슬라이스
    +t필드) — `ros2 bag info`로 정상 인식 확인(Duration
    39.998165375s, 파일명과 정확히 일치), 손상 없음, 스윕 재검증에
    재사용.
  - `src/agconav_test_worlds/config/path_calib_dummy.yaml` — 더
    이전 세션이 남긴 것으로 이번 작업과 무관(건드리지 않음).
- **GPS 포함 velocity bag** (`bags/velocity_4m_5mps_gps_attempt2`,
  이번 진단의 원본 데이터) — `ros2 bag info`로 재확인: Duration
  2254.092371115s, `/drone/gps` 22509건/`/drone/points` 22508건/
  `/tf` 112545건, 12개 mcap 파일 전부 정상 — **무사, 재비행
  불필요**(지시사항 4번 항목 확인 완료, 재비행 불필요하므로 별도
  보고 없이 진행).

### 0-1. 디스크 정리
`df -h ~` → 72%(21G 여유), 임계값(70%) 초과. 1순위(84m/GICP 원본)·
2순위(#5/R보정/디스큐 원본) bag은 `bags/`에 이미 없음(이전 세션들이
정리 완료, 확인만 함). 대신 위에서 발견한 `bags/_chunk_param`
(1.4GB, 크래시가 만든 쓰다 만 스크래치 파일, 원본에서 재생성 가능)을
삭제 → **70%(23G 여유)로 회복**, 목표 충족. GPS bag과
`run_results/glim_diag_dumps/sweep_*`(현재 진단 핵심 산출물)는
전혀 건드리지 않음. `bags/_slice40`(453MB)는 스윕 재사용 목적으로
보존(정리 안 함).

### 1. 가설 검증(1-1/1-2) — 직전(기록 안 된) 세션 결과 확인
- **1-1**: `run_results/glim_diag_dumps/gt_velocity_profile_0_30s.png`
  존재(PNG 유효성 확인됨) — 정지→전진 전환이 t≈15~17s 근처라는
  Phase 1의 진단(위 "GLIM odometry 발산 원인 진단" 절 1단계)과 일치.
  다시 그릴 필요 없음.
- **1-2**: `glim_config/config_odometry_ct.json`의
  `constant_velocity_inf_scale: 1e0` 확인, 바로 위 주석: "이 bag은
  드론이 이미 8 m/s로 순항 중일 때 시작한다(정지 구간 없음). CT
  오도메트리는 속도 0으로 초기화하므로, 1e3짜리 강한 등속 사전확률이
  실제 운동과 충돌해 첫 프레임부터 표류한다. 사전확률을 크게
  낮춘다." — **이 주석의 전제(정지 구간 없음)가 실측 GT(정지
  8.9~13.6s 이후 t≈15~17s 전진 전환)와 정면으로 어긋난다**는 것이
  Phase 1에서 이미 확인됨. 이 주석은 다른 실험(비행 속도/시작조건이
  다른)에서 복사돼 온 것으로 추정. **또한 이 주석 자체가 "CT
  오도메트리는 속도 0으로 초기화"라고 명시하므로, 2-2(초기속도 0
  명시 파라미터)는 별도로 찾을 필요 없이 이미 기본값이 0 —
  해당사항 없음으로 결론.**

### 2. constant_velocity_inf_scale 스윕 — 직전 세션 결과 정리 + 정량 재분석
직전(기록 안 된) 세션이 `run_results/diag_scripts/run_sweep_test.sh`
+ `make_slice_with_t.py`로 만든 40초 슬라이스(`bags/_slice40`)에
6개 값(1e-2, 1e-1, **1e0=기준값**, 1e2, 1e3, 1e4)을 이미 스윕
완료해뒀으나(위 손상검사 참고, 전부 정상 완주) **정량 비교 결과를
어디에도 저장하지 않은 채 중단됨**. `run_results/diag_scripts/
compare_gt_glim.py bags/_slice40 <dump_dir> <out> 5.0`을 6개 전부에
재실행(GLIM 재실행 없이 이미 있는 odom_imu.txt만 분석 — 저비용)해
정량 결과를 얻었다:

| constant_velocity_inf_scale | max error (40s 슬라이스) | 발산 시작(자동탐지) |
|---|---|---|
| 1e-2 | 113.17m | t=12.5s |
| 1e-1 | 168.11m | t=13.0s |
| **1e0(기준)** | 86.20m | t=16.2s |
| 1e2 | 38.94m | t=32.4s |
| 1e3 | 33.48m | t=14.2s |
| 1e4 | **32.40m(최소)** | **t=47.8s(사실상 미검출)** |

**핵심 발견 — 원래 가설(값을 낮춰라)과 정반대**: 값을 낮출수록
(1e-1, 1e-2) 오히려 더 크게 발산하고(168m, 113m > 기준 86m), 값을
높일수록(1e2, 1e3, 1e4) 뚜렷이 개선된다(39m→33m→32m, 1e4에서는 40초
윈도우 안에서 발산 시작점 자체가 사실상 검출 안 됨). **해석**:
`constant_velocity_inf_scale`은 "이 bag의 순항 속도"에 대한
사전확률이 아니라, **연속 프레임 간 속도 변화를 부드럽게 유지시키는
정규화 항의 가중치**다(config_odometry_ct.json 주석: "Weight for
constant velocity constraints"). 이 bag의 진짜 문제는 t≈15~17s
정지→전진 전환 구간에서 CT-GICP 프레임 간 정합이 불안정해지는
것인데, 이 정규화를 강하게 걸수록(가중치를 높일수록) 그 불안정한
튐을 억제해준다 — 기존 주석의 "정지 구간 없음" 전제가 애초에 틀렸기
때문에 그 주석이 권한 방향(낮추기)도 이 bag에는 맞지 않았던 것.

**1e5 추가 시도 — 결론 유보(진짜 불안정성 아님, 내 프로세스 관리
버그로 판명)**: 추세가 계속되는지(더 높이면 계속 좋아지는지, 아니면
반전되는지) 확인하려 `glim_config_cv1e5`/`glim_config_cv1e6`를
추가로 만들어 시도했으나, 첫 시도는 상대경로를 절대경로로 안 바꿔서
설정을 못 읽는 내 실수로 실패(`InvalidTopicNameError`로 즉시
abort), 재시도 두 번은 `pgrep -f "glim_ros/glim_rosnode" | tail -1`이
`ros2 run` 래퍼 프로세스와 실제 바이너리 자식 프로세스 중 어느 쪽을
잡을지 일정하지 않아(Phase 1에서 이미 한 번 겪은 것과 동일한 종류의
문제) `kill -INT`가 엉뚱한 PID로 가 실제 GLIM 프로세스는 안 죽고
방치됨 → "이미 죽었다"고 잘못 판단해 dump 회수도 실패로 오판. 방치된
두 프로세스(PID 4328, 4427)를 뒤늦게 발견해 `kill -9`로 정리함
(메모리 누수/자원낭비만 있었고 다른 실험에는 영향 없었음, `free -h`로
확인). **1e5/1e6가 실제로 불안정한지는 검증되지 않았다** — 이건
"GLIM이 1e5에서 불안정하다"는 발견이 아니라 순전히 이번 세션의
프로세스 관리 실수였음을 분명히 기록해둔다(다음 세션이 오해하지
않도록). 시간 예산상 재시도는 안 함 — 1e3/1e4 사이의 차이가 이미
작고(33.48 vs 32.40, ~3%), 지시사항이 요구한 "최소 2~3단계"는 이미
6단계로 충분히 초과 달성했다고 판단.

### 2-1. 최종 값 결정 — cv1e3 채택(cv1e4 대신)
40초 슬라이스만 보면 1e4가 근소하게 더 낫지만(32.40m vs 33.48m,
발산 미검출 vs t=14.2s), **cv1e3를 채택**한다. 근거: (1) 크래시 전
직전 세션이 이미 cv1e3로 전체 bag(2254s) 실행을 시작해 t=577.8s
(전체의 25.6%)까지 **내부 크래시 없이 정상 진행 중이었다**는 실측
증거가 있다(위 -1절, `fullrun_cv1e3_glim.log`) — 즉 cv1e3는 짧은
슬라이스뿐 아니라 더 긴 구간에서도 안정적이라는 근거가 이미 있는
반면 cv1e4는 40초 슬라이스에서만 검증됐다. (2) 1e3/1e4 성능 차이가
~3%로 작아 안정성 실적을 우선하는 게 안전한 기본값이라고 판단. (3)
1e5에서 (원인 불명확하지만) 초반 지속적인 IMU rewind 경고가 계속
쌓이는 등 정상 스윕들과 다른 거동을 보인 것도(비록 확정 결론은 아니지만)
너무 높은 값 근처는 피하는 쪽이 안전. `glim_config/
config_odometry_ct.json`은 건드리지 않고(이번 실험 전용 사본
정책 유지), `glim_config_cv1e3/`를 그대로 3단계 전체 재실행에 쓴다.

### 3. 전체 재실행(cv1e3) — 완료, ATE 계산까지 끝남

`run_results/diag_scripts/run_glim_chunked_param.sh`로 `glim_config_cv1e3`,
전체 12청크(2254s)를 처음부터 다시 실행(직전 세션의 중단된 시도는 dump가
전혀 없어 이어받기 불가능했으므로 재시작). 19:19:17 시작, 19:58경
`덤프 저장 완료`로 정상 종료(약 39분 소요, rate=1.0 실시간 재생과 거의
일치). 크래시/메모리가드 발동 없이 12청크 전부 완주 — **`traj_lidar.txt`
9011개 pose, t=[8.9, 2258.3]s로 bag 전체를 커버**(directly 확인,
`run_results/glim_diag_dumps/fullrun_cv1e3/`). 진행 중 메모리 최대
관측치는 GLIM 프로세스 RSS 3.35GB(가드 임계값 4.6GB 대비 여유 충분),
디스크는 청크별 변환 스크래치로 71~73% 사이에서 진동 후 종료 시
71%(22G 여유)로 복귀 — 전부 안전 범위.

`run_results/diag_scripts/ate_full.py`(신규, `ate_align.py`의 Umeyama
정합 로직을 GLIM TUM txt 입력에 맞게 이식 — **RESULTS.md 10절/84m 실험과
완전히 동일한 방법론**: 전체 궤적 매칭쌍으로 스케일고정 Umeyama 정합,
mean/median/p95/max ATE + 축별 RMSE)로 GT(GPS bag `/tf`) 대비 계산:

**1차 결과(bag 전체, 절단 없음): ATE 평균 116.46m, 중앙값 76.02m, 최대
774.30m — 목표(23m 근처) 대비 크게 못 미침.** 그러나 원인을 더 파보니
이 숫자 자체가 오염돼 있었다:

- `compare_gt_glim.py`로 시간대별 오차를 찍어보니 t≈2145s 이후 GT 자체가
  물리적으로 불가능한 값(z=-678m)으로 튀는 것을 발견. GT는 시뮬레이션의
  "진짜" 위치이므로 이게 이상해서 원인을 추적했다.
- `/drone/path_status`(1건)의 기록시각이 bag 시작 기준 **t≈2129.67s**임을
  `ros2 bag` 원시 타임스탬프로 확인. GT `/tf`를 t=2090~2150s 구간에서
  촘촘히 찍어보니 **t≈2129s부터 z가 초당 수십 m씩 떨어지는 자유낙하가
  시작**돼(t=2133.18s z=10.25m → t=2148.78s z=-483.54m) 결국
  (-117.34, 511.97, -678.65)에서 멈춘다.
- **결론: 이건 GLIM/디스큐 버그가 아니라 시뮬레이션 자체의 정상적인
  종료 시퀀스다.** `path_status=true`(임무 완료) 발행 직후 follower가
  속도 명령을 끊고, 드론이 호버링/착륙 모드로 전환되지 않은 채 그대로
  중력에 맡겨져 맵 바깥 어딘가로 떨어져 정지한 것으로 보인다(추측이지만
  타이밍이 t=2129.67s로 정확히 일치해 근거는 충분). 이 bag의 "비행
  시간" 지표(2254.09s, 위 4단계 기록)는 이 후처리(임무완료 후
  자유낙하+정지, ~125초)까지 포함하고 있었다는 뜻이지만, **이번
  세션에서 그 지표 자체를 고치는 것은 범위 밖**이라 손대지 않음
  (참고용으로만 기록).

**2차 결과(t<=2129s로 절단, 실제 비행 구간만): ATE 평균 68.60m, 중앙값
72.66m, p95 124.64m, 최대 137.30m, 축별 RMSE X 48.37/Y 57.02/Z 20.03m.**
`run_results/glim_diag_dumps/fullrun_cv1e3/ate_gt_vs_glim.png`(신규,
`ate_full_plot.py`)로 궤적/시간별 오차를 시각화했다:

- 궤적 그림: GLIM(빨강, 정합됨)이 GT(초록, 촘촘한 왕복 스트립 커버리지
  패턴)의 **박스 형태를 대략 따라가지만**, 위쪽(비행 시작 직후 구간)과
  아래쪽(비행 후반부)에서 GT 박스를 크게 벗어나 뒤엉킨 궤적을 그린다.
- 시간별 오차 그림: **단조 발산이 아니라 각 스트립 왕복마다 주기적으로
  진동**(저점 5~30m ↔ 고점 90~140m). 저점은 스트립 중간의 직진 구간과,
  고점은 스트립 끝 U턴 구간과 거의 일치하는 패턴으로 보인다 — **잔여
  오차의 상당 부분이 방향전환(U턴) 시 CT-GICP 정합 불안정에서 온다**는
  가설을 뒷받침(1단계 진단의 "정지→전진 전환에서 불안정"이라는 근본
  성격이 U턴에서도 반복되는 것으로 해석됨, 다만 확정 검증은 범위 밖).

### 4. 종합 평가 — 부분 성공(근본 버그는 고쳤으나 목표 수치 미달)

**가설(1순위 후보: `constant_velocity_inf_scale`)은 확인됐고 수정도
효과가 있었다** — 원래 문제였던 "t≈15~17s부터 물리적으로 불가능한 값
(수백 m 단위)으로 발산"이라는 **치명적 발산은 완전히 사라졌다**(cv1e3
전체 궤적이 bag 끝까지 유한하고 GT 박스 근방에 머무름, 이전 4회 시도
전부 나타났던 "물리적으로 불가능"이라는 표현에 해당하는 값은 이제 없음).
다만 **잔여 드리프트(ATE 평균 68.6m)가 목표(84m 실험 수준 ATE~23m
근처)에는 못 미친다** — 지시사항 3단계의 "성공" 기준(23m 근처 이하)을
엄밀하게는 만족하지 못했다.

**Phase 4(GPS)로 넘어가는 것을 권고한다 (지시사항이 예시로 든 "2순위
후보: 디스큐 설정"보다 이쪽을 우선 권고)** — 근거:
1. 시간별 오차 그림이 보여주듯 잔여 오차가 **경계가 있고(bounded,
   5~140m 사이에서 진동) 주기적**이지, 이전처럼 통제 불능으로 계속
   커지는 발산이 아니다. 이런 패턴은 GPS 사후결합(이미 3-3절에서 설계,
   `run_results/glim_gps_correct.py`로 구현까지 끝나 있음 — GPS
   앵커(10Hz)로 주기적으로 스냅 + 앵커 사이 SE3 보간)이 정확히 잘 듣는
   유형의 오차다. 반면 원래(수정 전) 궤적처럼 "혼란스러운 스크리블"
   수준으로 완전히 망가진 궤적에는 GPS 스냅을 적용해도 모양 자체가
   의미 없어 소용이 적었을 것(6절 4차 시도 결론과 일관).
2. "디스큐 설정"(지시사항이 예로 든 2순위 후보)은 이번 세션에서 조사한
   범위(1단계 ⑤ 디스큐 on/off 비교, 위 "GLIM odometry 발산 원인 진단"
   절)에서 이미 "2차 악화요인일 뿐 1차 원인이 아님"으로 결론 내린
   축이라 이걸 다시 붙잡는 것은 이미 확인한 결론을 뒤집는 것 — 반면
   GPS 사후결합은 애초에 이 실험 트랙의 최종 목표(위 "[새 세션]
   GLIM(+GPS 사후결합) 전체 파이프라인 실험" 절)였고 코드도 이미
   준비돼 있어 다음 단계로 자연스럽다.
3. 다만 **이번 세션 시간 예산(2시간)이 이미 거의 소진**돼 Phase 4를
   직접 실행하지는 않았다(정지조건: "확정 범위 밖 새 선택지 필요시
   멈추고 보고" — GPS 파이프라인 실행은 이번 세션에 명시적으로
   합의되지 않은 새 단계라 판단해 다음 세션/사용자 확인으로 넘김).

### 5. 저장/보고
`glim_config_cv1e3/`(채택된 설정, git에 포함)와 `glim_config_cv1e-1/
_cv1e-2/_cv2/_cv1e4/_cv1e5/_cv1e6`(스윕 과정 전부, 재현성 위해 보존,
전부 작은 JSON 파일이라 용량 문제 없음)를 그대로 커밋 대상에 포함.
`run_results/glim_diag_dumps/{sweep_*,fullrun_cv1e3}/`,
`run_results/logs/{sweep_*,fullrun_cv1e3}_*.log`,
`run_results/diag_scripts/{gt_velocity_profile,make_slice_with_t,
ate_full,ate_full_plot}.py`, `run_results/diag_scripts/{run_sweep_test,
run_glim_chunked_param}.sh` 전부 신규 커밋 대상. `bags/_slice40`은
스크래치 성격(원본에서 재생성 가능, 22G 원본과 별개로 디스크 차지)이라
**git에는 안 올리고(gitignore 대상 확인) 디스크에만 유지**(다음 세션이
스윕을 더 하고 싶으면 재사용 가능, 필요 없으면 다음 세션이 정리 판단).

### 세션 재개용 참고 (다음 세션이 이어받을 경우)
1. **다음 최우선 작업은 Phase 4(GPS 사후결합)다** — 설계(3-3절)와
   구현(`run_results/glim_gps_correct.py`)이 이미 끝나 있고, 이번
   세션이 만든 `run_results/glim_diag_dumps/fullrun_cv1e3/traj_lidar.txt`
   (9011 pose, cv1e3 설정, 치명적 발산 없음)를 입력으로 바로 쓸 수
   있다. GPS 좌표변환(`pymap3d`, world 기준점 3-2절)도 이미 검증
   완료. 남은 건: (a) GPS 사후결합 적용, (b)
   `run_results/glim_gps_build_map.py`로 지도 생성, (c)
   `run_results/glim_gps_metrics.py`로 coverage/wheel FN%/leg FN% 등
   지표 산출, (d) 최종 ATE가 GPS 보정으로 목표(23m 근처)에 도달하는지
   재확인.
2. **주의**: 이번 세션이 발견한 "bag 뒷부분(t>2129.67s) 자유낙하"는
   GPS 사후결합 스크립트에도 영향을 줄 수 있다 — GPS 앵커도 이 구간의
   GPS 샘플(진짜 GPS 센서라면 자유낙하 중에도 계속 발행됐을 것)을 그대로
   신뢰하면 안 됨. Phase 4 진행 시 GPS/GLIM 입력 모두 t<=2129.67s로
   절단하고 시작하는 것을 권장(이번 세션이 이미 그 경계를 확인해둠).
3. 지도 생성/지표 산출 후 `run_results/SUMMARY.md`에 이번 GLIM+GPS
   트랙 전체 결과를 정리하고, 5m AGL 방법B 실험(위쪽 섹션들)과 같은
   형식의 비교표에 추가하는 것도 다음 세션 범위.

## [새 세션] GLIM(+GPS 사후결합) 전체 파이프라인 실험

**전체 타임아웃 4시간.** 판단 지점은 확인 없이 스스로 진행, 근거는 이
문서에 계속 기록. 목표: 우리 자체 칼만필터/GICP 대신 GLIM(GLIM의
odometry+sub_mapping+global_mapping 전체)을 GPS로 드리프트 억제한
상태로 돌려서, wheel FN%·최대연결덩어리% 등 4개 지표를 산출.
`drone_elevation_mapper.py`는 절대 안 건드림(다른 실험에서 계속 쓰임).

### 0. 재개 확인
`pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음(깨끗). `df -h ~` →
69%/23G 여유(안전, 0-1단계 별도 정리 불필요 — 아래 참고). `git log`
HEAD가 직전 세션 커밋(`3f5700d`)과 일치, 이번 작업 전혀 시작 안 된 상태.

### 0-1. 디스크 정리 판단
지시사항은 70% 이상이면 정리하라고 했는데 현재 69%로 임계값 아래다 —
**정리 스킵**(애매한 걸 지우지 말라는 지시와도 부합, 여유가 이미
충분). 4단계(velocity 재비행) bag 기록 중 60초 주기 감시로 실시간
대응하는 것으로 충분하다고 판단.

### 1. 사전 확인
- GLIM PPA 설치 확인: `dpkg -l | grep glim` → `ros-jazzy-glim`,
  `ros-jazzy-glim-ros` 1.2.2-0noble arm64 정상 설치 확인(재설치 안 함).
  `ros2 pkg executables glim_ros` → `glim_rosbag`/`glim_rosnode`/
  `map_editor`/`offline_viewer`/`validator_node` 정상 조회.
- `path_100x100_5m_4m_5mps.yaml`, `Seongdong_gu_100x100_dynamic` 월드
  둘 다 존재 확인(`find` 명령으로). 재생성/드라이런 스킵(지시대로).
- `glim_config/`(config.json이 CT+passthrough+pose_graph 조합 고정,
  이유는 파일 내 주석에 상세 기록돼 있음 — GPU 없음+IMU 회전 감지
  안정성) 그대로 존재, 이번에도 재사용.
- **중요 발견**: `src/agconav_test_worlds/scripts/add_point_times.py`
  (기존 GLIM 실험이 만든 후처리 스크립트) 발견 — gpu_lidar가 점별
  타임스탬프를 안 주므로, organized cloud의 열(column) 인덱스로
  `t = col/width * 0.1`를 계산해 PointField `t`를 추가해준다(우리가
  ④ 디스큐에서 쓴 것과 완전히 동일한 근사식). GLIM은 이 필드가 있으면
  `autoconf_perpoint_times`로 자동 인식한다(`glim_config/
  config_sensors.json` 주석 확인). **이번에도 velocity 재비행 후 이
  스크립트로 후처리해 GLIM에 넣을 것.**

### 2. GLIM GPS/GNSS 결합 지원 조사
- 설치된 apt 패키지(`ros-jazzy-glim`, `ros-jazzy-glim-ros`)의 `.so`
  목록(`dpkg -L`)에 gnss/gps 관련 라이브러리 없음(전부 확인:
  `libglim`, `libglobal_mapping[_pose_graph]`,
  `libodometry_estimation_{cpu,ct}`, `libsub_mapping[_passthrough]`,
  `libmemory_monitor`, `lib{standard,interactive,rviz}_viewer`,
  `libmap_editor`).
- GLIM 코어(`koide3/glim`) 공식 README/문서(WebFetch) — GPS/GNSS를
  지원 입력으로 명시한 곳 없음. "global callback slot" 메커니즘으로
  factor graph에 커스텀 제약을 넣을 수 있다는 확장 포인트만 있음.
- **`koide3/glim_ext`**(별도 저장소, WebSearch+WebFetch로 확인) —
  `libgnss_global.so`("GNSS-based constraints for global optimization")
  모듈이 실제로 존재. 단 (1) 저장소 README가 명시적으로 "ROS2 only"
  + **"half-baked code that may not be well-maintained and not
  suitable for practical purposes"**라고 경고, (2) apt 패키지에는
  전혀 포함 안 돼 있어 **별도로 이 저장소를 clone+빌드**해야 함(ARM64
  빌드 성공 여부, 의존성, 우리 config와의 정합성 전부 미검증).
- **판단: glim_ext의 gnss_global 미채택, 사후 결합(loose coupling)
  방식 채택.** 근거: (1) 공식적으로 "실용적이지 않다"고 경고된
  실험적 코드를 4시간 예산 안에 빌드+검증+디버깅까지 하는 것은 위험이
  지시사항의 "안전한 기본값" 원칙에 안 맞음, (2) 지시사항 자체가
  "지원 안 되면 사후 결합을 직접 구현해라"는 구체적 폴백 알고리즘을
  이미 명시해뒀음 — 이게 이런 상황(네이티브 지원은 있지만 프로덕션
  품질이 아님)을 위한 안전한 기본값으로 판단.

### 2-1. GPS 센서/토픽 확인 및 브리지 추가
- `agconav_description`과 `agconav_test_worlds`(dynamic 모델) 양쪽
  `model.sdf`에 `navsat_sensor`(topic=`drone/gps`, update_rate=10Hz —
  지시사항의 "GPS 주기 10Hz"와 일치, `<noise>` 블록 없음) 존재 확인.
  지금까지의 실험은 이 센서를 브리지/기록한 적이 없었음(GPS를 쓴 첫
  실험).
- `ros_gz_bridge`가 `sensor_msgs/msg/NavSatFix`↔`gz.msgs.NavSat`
  변환을 지원함을 설치된 헤더(`convert/sensor_msgs.hpp`)에서 확인.
- `src/agconav_test_worlds/launch/experiment.launch.py` 수정(2곳):
  1. `exp_sensor_bridge`에
     `/drone/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat` 추가.
  2. `recorder`(ros2 bag record) 토픽 목록에 `/drone/gps` 추가.
  `drone_elevation_mapper.py`는 이 수정과 무관(건드리지 않음).
  symlink-install이라 재빌드 불필요(`readlink`로 launch 파일이 src를
  직접 가리킴 확인), `py_compile`로 문법만 확인.

### 3. GPS 사후 결합(loose coupling) 설계 확정
- GLIM을 먼저 GPS 없이 그대로 돌려 원시 궤적(각 스캔 시점의
  map/odom→imu 또는 base pose)을 얻는다(`glim_rosbag` bag 배치처리,
  `librviz_viewer`가 발행하는 TF/odometry 토픽에서 궤적 추출 — 구체
  방법은 4단계에서 실측 확인 후 기록).
- GPS(`sensor_msgs/NavSatFix`, WGS84 위경도고도)를 로컬 ENU/map
  좌표로 변환해야 한다 — 시뮬레이션 world 원점의 위경도 기준점(GLIM/
  robot_localization의 navsat_transform_node 관례상 필요)을 확인해야
  함(4단계에서 실측).
- 매 GPS 샘플 시각(10Hz)마다 그 시각의 GLIM 궤적을 보간해 얻고, GPS
  절대위치로 스냅(place) → 이후 다음 GPS 앵커까지 구간은, "GLIM이
  추정한 상대 이동(모양)은 유지하되 두 앵커 사이를 보정값 기준으로
  재배치"한다. 구현: 각 앵커 구간 [t0,t1]에서 GLIM 원시 pose와 보정된
  앵커 pose 사이의 잔차(보정 오프셋, SE(3))를 t0/t1에서 각각 계산하고,
  그 사이 임의 시각 t의 pose는 GLIM 원시 pose에 (t0→t1 오프셋을
  시간비례로 SE(3) 보간한 보정치)를 곱해 적용 — 회전은 slerp,
  평행이동은 lerp(지시사항 그대로).
- 다음: 4단계(velocity 재비행)로 진행.

### 3-1. GLIM 궤적 출력 방식 확인(WebFetch, 공식 문서)
GLIM(`glim_rosnode`/`glim_rosbag`)은 종료 시(auto_quit 또는 수동 종료)
`/tmp/dump`에 TUM 포맷(`t x y z qx qy qz qw`, 한 줄에 한 pose) 궤적
파일 4개를 자동 저장한다: `odom_imu.txt`/`odom_lidar.txt`(루프클로저
없는 순수 오도메트리), `traj_imu.txt`/`traj_lidar.txt`(글로벌 매핑
결과, pose_graph 루프클로저 적용됨 — 우리 config가
`enable_global_mapping: true`이므로 이 파일이 최종 산출물). **점을
map으로 옮기려면 라이다 원점 pose가 필요하므로 `traj_lidar.txt`를
원시 궤적으로 채택**(IMU→LiDAR 변환 재계산 불필요).
[glim quickstart](https://koide3.github.io/glim/quickstart.html) 참고.

### 3-2. GPS 좌표 변환 확인
- `Seongdong_gu_100x100_dynamic.world`의 `<spherical_coordinates>`:
  lat=37.54233814881853°, lon=127.06050643805561°, elevation=15.4m —
  이게 world 원점(0,0,0)에 대응하는 WGS84 기준점(gz-sim 표준 관례,
  world x=East, y=North, z=Up로 정렬).
- `pymap3d`(WGS84 geodetic↔ENU 변환) 설치 필요 확인 →
  `pip install --user --break-system-packages pymap3d` 성공(3.2.0).
  이유: 정확한 타원체 기반 변환이 필요해 직접 근사식을 짜는 것보다
  검증된 라이브러리가 안전. `--break-system-packages`는 로컬 사용자
  패키지 설치일 뿐이라 위험도 낮다고 판단.
- 실제 gz-sim navsat 센서가 이 변환과 정확히 일치하는지는 bag의 실측
  GPS 값과 동시각 `/tf`(map→base_link, 물리엔진의 진짜 위치)를
  대조해서 4단계 완료 후 검증할 것(아직 미검증).

### 3-3. GPS 사후 결합 알고리즘 확정
GLIM 원시 궤적(SE3, "모양"을 담고 있음)에 시간에 따라 부드럽게 변하는
"보정 transform" C(t)를 왼쪽에서 곱해 GPS 앵커에 맞춘다:
```
P_raw(t)      = GLIM traj_lidar.txt에서 시각 t의 raw pose (SE3)
C_k           = GPS 앵커 시각 t_k에서의 보정 transform
              = translation: GPS_position(t_k) - P_raw(t_k).translation
              = rotation: identity (GPS는 orientation을 안 주므로 —
                안전한 기본값, 회전 보정 없음)
C(t), t_k<=t<=t_{k+1}:
              = translation: lerp(C_k.t, C_{k+1}.t, alpha)
              = rotation: slerp(C_k.R, C_{k+1}.R, alpha)  (둘 다
                identity라 결과도 identity — 하지만 지시사항대로 SE3
                보간 함수 자체는 일반적으로 구현해 향후 회전보정 필요
                시에도 그대로 재사용 가능하게 함)
              alpha = (t - t_k) / (t_{k+1} - t_k)
P_corrected(t) = C(t) @ P_raw(t)   (C(t)는 world-frame 보정이므로 왼쪽 곱)
```
회전 보정을 항상 identity로 고정하는 이유: NavSatFix는 3D 위치만 주고
orientation이 없다 — 임의로 회전을 추정하려 들면(예: GPS 이동 방향에서
yaw 추정) 저속/정지 구간에서 노이즈가 커지고 이번 실험 범위를 벗어나는
과설계다. "안전한 기본값"으로 판단.

### 3-4. 재사용 가능한 기존 스크립트 확인
- `run_results/gt_traversable.py` — GT 통과가능 마스크(wheel/leg,
  건물풋프린트+높이차 기준) 그대로 재사용 가능(무수정).
- `run_results/build_methodB_cloud.py` — bag의 point cloud를 스트리밍
  으로 읽어 pose 적용 후 디스크에 바로 append하는 메모리 안전 패턴
  (이 VM 5.8GB RAM에서 21771 스캔 전체를 리스트에 들고 있다가
  concatenate하면 OOM-kill됨을 실측한 교훈) — 이번 GLIM+GPS 지도
  생성 스크립트도 이 패턴을 그대로 따른다(TF 조회 대신 GLIM+GPS 보정
  pose로 대체).
- `run_results/capture_nav_fn.py` — coverage_percent 정의(n_valid/
  n_cells*100) 재사용.

### 4. Velocity 재비행(GPS 포함) — 1차 시도 디스크 부족으로 중단, 정리 후 재시도
1차 시도(`gps_attempt1`): wp 1836/5251(806s, 약 35%)에서 디스크
80%(59G/78G) 도달 → 감시 스크립트가 정상적으로 감지해 SIGINT→(100초
후)SIGTERM으로 graceful shutdown, 잔여 프로세스 없음 확인. **버그
아님, 순수 용량 문제**: 시작 시점 69%(23G 여유)로 "정리 불필요"라고
판단했었는데(0-1절), 새로 만드는 GPS bag이 이전 실험(22GB)과 비슷한
크기가 될 것을 過小평가했다 — 80% 안전마진까지 감안하면 실제 쓸 수
있는 여유는 23G가 아니라 ~8.6G뿐이었다(78G*(80-69)% ≈ 8.6G).

**정리(사용자 지시 0-1절 그대로 적용)**:
- 실패한 `bags/velocity_4m_5mps_gps_attempt1`(8.9G, 불완전 — 재사용
  불가) 삭제.
- `bags/velocity_4m_5mps_attempt1`(22G) + `_tf_cache.pkl`(24M) 삭제 —
  근거: 이 bag을 쓴 실험(#5 기준선, A/①R보정, B(구)/B'(신)④디스큐)
  전부 완료돼 `run_results/SUMMARY.md`에 최종 결과 기록됨(grep으로
  "10.91%"/"20.51%" 등 5곳 확인) + `git log origin/test_main_brian`
  HEAD가 로컬과 동일한 `3f5700d`로 이미 push 확인됨 — 재현이 필요하면
  재비행 가능하지만 이번 세션 범위 밖.
- 지우지 않은 것: `exp_teleport`(용도 불명, 애매해서 보존), Docker
  이미지/GICP 캐시(정리 없이도 이미 45G 여유 확보돼 불필요 — 애매한
  건 지우지 말라는 지시에 따름).
- 결과: 39%(29G/78G, 45G 여유)로 회복. 재시도 진행.

**2차 시도(`gps_attempt2`) — 정상 완주.** launch 03:12:39 시작, 03:48:42
`path_status=true` 확인, graceful shutdown(SIGINT 후 100초 넘어 SIGTERM
에스컬레이션 — 예상된 대용량 bag 정상 동작), 잔여 프로세스 없음. bag
디렉토리에 `metadata.yaml` 누락(지시사항이 미리 경고한 SIGTERM
에스컬레이션발 버그, 예상대로 재현) → `ros2 bag reindex`로 즉시 복구,
정상 확인:
- **Duration: 2254.092371115s** (≈37.57분) — 이걸 "비행 소요시간"
  지표로 채택(launch~path_status 전체 36분3초에는 world 로딩 등
  비행과 무관한 오버헤드가 섞여 있어, bag 기록 자체의 duration이 더
  정확한 "비행 시간"으로 판단. 이전 GPS 없는 실험의 동일 경로/속도
  bag duration 2258.8s와 거의 일치 — 재현성 확인).
- 토픽: `/drone/gps` 22509건(NavSatFix), `/drone/points` 22508건,
  `/drone/imu` 224866건, `/tf` 112545건 — **GPS와 LiDAR가 거의 1:1로
  매칭**(둘 다 10Hz 설계와 일치), 브리지/기록 정상 확인.
- 최종 디스크: 69%(51G/78G) — 안전.
- 다음: 5단계, `add_point_times.py`로 이 bag에 point-level t 필드
  추가 후 GLIM 실행.

### 5. point-level t 필드 추가 — 디스크 부족으로 전략 변경
`add_point_times.py bags/velocity_4m_5mps_gps_attempt2
bags/velocity_4m_5mps_gps_attempt2_t`로 새 bag을 통째로 만들려다
디스크가 69%→88%(9.2G 여유)까지 차오르는 것을 60초 감시 스크립트가
잡아 강제 종료(불완전 결과물 `..._t` 즉시 삭제, 원본 gps_attempt2
bag은 무사 — 23G 그대로). **원본(23G)과 신규(point_step 32→36B라
point cloud만 12.5%↑, 다른 토픽은 그대로 — 최소 25G+ 예상)를 동시에
담을 공간이 이 VM(78G, 여유 23G)엔 없다.**

**전략 변경 — 디스크에 중간 bag을 아예 안 만든다**:
`run_results/points_t_republisher.py`(신규, add_point_times.py의
`add_times()` 로직 그대로 재사용) — `/drone/points`를 구독해 t필드를
얹어 `/drone/points_t`로 즉시 재발행하는 노드. 파이프라인을
`ros2 bag play`(원본 재생, 디스크 추가 소비 없음) → 이 republisher →
`glim_rosnode`(라이브 구독 모드, `points_topic:=/drone/points_t`로
오버라이드)로 바꾼다 — `glim_rosbag`(bag 파일을 직접 배치 처리하는
모드)는 t필드가 이미 파일에 있어야 하므로 이번엔 못 쓰고, 라이브
모드로 전환. 대신 자동 재생속도 조절(`glim_rosbag`의 장점) 없이
`ros2 bag play --rate 1.0`(실시간)으로 안전하게 진행 — 4코어 VM에서
빠른 배속은 GLIM이 못 따라가 드랍될 위험이 있다고 판단.

**실측 결과 — rate=1.0도 못 따라감, rate=0.5로도 부족**: 첫 시도(rate
1.0)에서 `points/imu timestamp rewind detected`(시간이 거꾸로 감지)가
계속 반복. 원인 분리를 위해 republisher 없이 원본 `/drone/points`를
GLIM에 직결(pseudo timestamp 폴백)해 재현한 결과 **rewind 0건** →
republisher 쪽 문제로 특정. republisher에 단조증가 가드(직전 발행
stamp보다 뒤로 가는 메시지는 버림, 근본원인 미상이나 안전하게 회피)를
추가해 rate=0.5로 재시도 → rewind는 5건까지 급감(사실상 해결)했지만
`large time gap between consecutive LiDAR frames`(diff 0.6~1.0s, 스캔
간격 0.2s의 3~5배)는 여전히 반복 — **CT odometry 연산 자체가 스캔당
약 0.8~1.2초 걸리는 것으로 추정**(diff 누적 속도로 역산). 이 페이스면
22508스캔 전체 처리에 최소 5~6시간 필요 — 이번 세션 전체 예산(4시간)을
이미 초과하는 작업량.

**시간 예산 재판단**: `glim_rosbag`(자동 속도조절)도 처리 자체를
빠르게 하진 않는다(정확성만 보장, CT 연산량은 동일 — 속도 문제의 근본
해법 아님). `config_odometry_cpu.json`(VGICP+IMU, 이번 velocity bag은
IMU 회전이 정상이라 이론상 쓸 수 있음)으로 바꾸면 더 빠를 수도 있으나,
남은 시간에 검증 안 된 새 조합으로 갈아타는 리스크가 커 **채택 안
함**(안전한 기본값 원칙).

**결정**: config는 그대로(CT+passthrough+pose_graph) 유지. 디스크
문제는 mcap 청크(12개, 각 ~2.5GB) 단위로 하나씩 t필드 변환→재생→삭제
순환으로 우회(`run_results/add_point_times_single.py`,
`run_results/run_glim_chunked.sh` 신규). GLIM은 라이브 노드로 청크
전체에 걸쳐 상태(궤적)를 유지한 채 이어 처리한다. **처리속도 자체는
못 올리므로, rate=1.0(재생 자체는 실시간)으로 진행해 전체 bag을 최대한
빨리 "훑되", GLIM이 못 따라가 밀리는 스캔은 QoS(depth=10, KEEP_LAST)로
자연 드랍되게 둔다** — 궤적 시간해상도가 낮아지는 대가를 감수하고
시간 안에 전체를 한 번은 지나가는 것을 우선한다(안전한 기본값 —
"느리지만 정확"보다 "이번 세션 예산 안에 끝나는 결과"를 택함, 품질
저하는 SUMMARY.md에 한계로 명시할 것). 남은 시간이 부족해지면 그
시점의 `/tmp/dump` 부분 궤적으로 마무리하고 다음 세션에 인계.

### 6. GLIM 실행 — 시도 4회, 전부 실패 (세션 종료, 다음 세션 인계)

**1차(전체 파이프라인 그대로, local+global mapping 둘 다 켬)**: 12청크
전부 재생 완료(총 ~38분)까지는 갔으나, `libmemory_monitor`가 "CPU
memory usage: 5851.95/5894.34 MB 99.28%"를 마지막으로 찍고
**OOM killer에 SIGKILL당함**(`[ros2run]: Killed`) — `/tmp/dump` 자체가
생성 안 됨(SIGKILL은 정상 종료 훅을 못 부름), **완전 손실**. sub_mapping
(passthrough)/global_mapping(pose_graph)이 서브맵을 계속 누적하며
메모리가 무계한으로 자라는 것으로 추정.

**2차(odometry-only, `enable_local_mapping:=false
enable_global_mapping:=false`)**: 메모리 증가는 여전했으나(청크당
~0.4~0.7GB) 미리 건 메모리 가드(사용량 4600MB 도달 시 SIGINT, 신규
`run_results/glim_memory_guard*` 패턴)가 05:05:45에 정상 발동해 dump는
받음(`traj_lidar.txt` 3016 pose, t=[9.4, 738.3]s — 전체의 32.8%만
처리). **그러나 궤적이 처음부터(t=136.5s) 이미 물리적으로 불가능한
값으로 발산**(x=-123, y=-95, z=-102m — 5m AGL 비행인데 z가 -100m대) —
로컬 참조(sub_mapping)나 전역 최적화(global_mapping) 없이 CT
오도메트리 단독으로는 이 데이터에서 안정적이지 않다는 뜻으로 해석.

**3차(local mapping만 켬, `enable_global_mapping:=false`)**: 메모리
가드 05:19:42 발동, dump 받음(`traj_lidar.txt` 2934 pose,
t=[9.1,648.1]s, 28.8%). **역시 t=106.4s부터 이미 발산**(z=153m) —
local_mapping(sub_mapping)만으로는 발산을 못 막음. **global_mapping
(pose_graph, 전역 최적화/루프클로저)이 발산 억제의 핵심이었다는 뜻** —
그런데 이걸 켜면 정확히 그것 때문에 메모리가 못 버틴다(1차 결과)는
딜레마.

**4차(1차와 동일 설정, 메모리 가드만 추가)**: 가드 05:35:24 발동, dump
받음(`traj_lidar.txt` 3065 pose, t=[9.1,744.4]s, 33.0%). **이번에도
t=101.3s부터 발산**(z=2.9→50→226→265m로 계속 커짐). global_mapping을
켰는데도 발산했다는 것은, 3차 결과("global_mapping이 핵심")라는 해석이
**틀렸거나 불충분함을 시사** — 진짜 원인은 아직 미상. 유력한 남은
가설(미검증, 시간 부족으로 조사 못함): `config_odometry_ct.json`의
`constant_velocity_inf_scale`/초기화 관련 주석("이 bag은 드론이 이미
8m/s로 순항 중일 때 시작한다는 가정으로 조정됨")이 이번 5m/s bag의
실제 시작 조건(호버링 후 가속 — 이전 실험들과 동일 경로이므로 정지
상태로 시작할 가능성이 높음)과 안 맞아 CT의 등속 사전확률이 초반부터
어긋나며 발산을 유발했을 가능성.

**결론 — 이번 세션에서는 GLIM으로 신뢰할 수 있는 궤적을 못 얻었다.**
4번의 시도(파이프라인 조합 3가지 × 메모리 가드) 전부 궤적이 물리적으로
불가능한 값으로 발산했고, 유일하게 발산 안 한 조합(1차, 전체
파이프라인)은 대신 메모리 부족으로 완전히 죽어 궤적 자체를 못 건졌다.
**GPS 사후 결합(3-3절 알고리즘)은 이미 구현해뒀지만
(`run_results/glim_gps_correct.py`), 입력으로 쓸 만한 정상 궤적이
없어 실행하지 못했다** — 발산한 궤적에 GPS 위치 보정을 적용해봐야
"모양은 유지, 위치만 절대좌표로 스냅"하는 방식이라 원본이 이미 망가진
상태에서는 의미 있는 결과가 안 나온다(시도 안 함, 시간 낭비 방지).

### 세션 재개용 참고 (다음 세션이 이어받을 경우)
1. **GPS 포함 bag은 정상 확보돼 있다** — `bags/velocity_4m_5mps_gps_attempt2`
   (23G, `ros2 bag reindex` 이미 완료, `/drone/gps` 22509건/`/drone/points`
   22508건 확인됨) — **재비행 불필요**, 이걸 그대로 재사용.
2. 다음 세션이 우선 시도해볼 것(시간순 우선순위):
   a. `config_odometry_ct.json`의 초기화/사전확률 파라미터를 이 bag의
      실제 시작 속도에 맞게 재조정(위 "유력한 남은 가설" 참고) —
      `glim_config/`는 이번 실험 전용이라 수정해도 안전.
   b. 그래도 발산하면 `config_odometry_cpu.json`(VGICP+IMU 타이트
      커플링, 이번 실험처럼 IMU 회전이 정상인 bag에선 쓸 수 있음 —
      PROGRESS.md 5단계에서 시간 부족으로 미시도했던 대안)로 전환
      시도. 단 sub_mapping/global_mapping도 이 조합에 맞는
      `config_sub_mapping_cpu.json`/`config_global_mapping_cpu.json`
      으로 함께 바꿔야 할 가능성 높음(미검증).
   c. 메모리 문제는 `libmemory_monitor`가 자체 정리를 안 하는 것으로
      보이므로, 궤적이 안정된 조합을 찾은 뒤에도 청크+가드 방식
      (`run_results/run_glim_chunked.sh` + 메모리 가드 패턴, 이번
      세션에서 검증된 안전장치)을 계속 쓰는 게 안전.
3. 준비된 후속 스크립트(전부 이번 세션에서 작성, 정상 궤적만 있으면
   바로 쓸 수 있음): `run_results/glim_gps_correct.py`(GPS 사후결합),
   `run_results/glim_gps_build_map.py`(지도 생성),
   `run_results/glim_gps_metrics.py`(4개 지표 중 3개 — 커버리지/wheel
   FN%/wheel·leg 최대연결덩어리%). **비행 소요시간은 이미 확정
   가능**(4단계, bag duration 2254.09s ≈ 37.57분).
4. `src/agconav_test_worlds/launch/experiment.launch.py`의 GPS
   브리지/기록 추가(2-1절)는 이미 커밋 대상 — 재작업 시 다시 안 해도 됨.

## [새 세션] ④ 디스큐 커버리지 손실 버그 수정 + B' 재측정

**전체 타임아웃 2시간(재비행 없음, 재처리만).** 판단 지점은 확인 없이
스스로 진행, 근거는 이 문서에 기록.

### 0. 재개 확인
`pgrep -af "ros2|gz sim|docker"` → dockerd 데몬 외 잔여 없음(깨끗).
`df -h ~` → 69%/24G 여유, 안전. `git status` → 이전 세션이 남긴
`src/agconav_test_worlds/config/path_calib_dummy.yaml`(이번 작업과 무관,
건드리지 않음) 외 깨끗. `git log` HEAD가 직전 세션 커밋(`78db75a`)과
일치 — 이번 작업(④ 재수정)은 전혀 시작 안 된 상태에서 재개.

### 1. 원인 진단 (추측 아님, 실측)
`ros2 run agconav_drone drone_elevation_mapper --ros-args ... -p
deskew_enabled:=true --log-level drone_elevation_mapper:=debug`로 짧은
bag 슬라이스(5x 재생, 진단 목적)를 흘려보내 DEBUG 로그를 직접 수집했다.
실제 에러 메시지(예시):

```
[DEBUG] ... deskew 구간 8/12 TF 조회 실패(이 구간만 버림):
Lookup would require extrapolation into the future.
Requested time 8.270833 but the latest data is at time 8.180000,
when looking up transform from frame [drone/os1_lidar] to frame [map]
```

`tf2_ros.TransformException`의 구체 타입은 extrapolation(미래 시각 요청)
— 정확히 예상한 그 원인이 맞았다. 실패 버킷 분포(진단 세션 전체
1170건 집계): `8/12`(354), `7/12`(340), `6/12`(305), `5/12`(69),
`9/12`(48), `4/12`(27), `3/12`(18), `2/12`(9) — **버킷 6~9(스캔
중후반부)에 압도적으로 집중**, 0/1/10/11은 전무. 원본 가설("맨 끝
버킷만 실패")과는 분포가 다소 다르지만("끝에서 두 번째 근방이 최다"),
근본 원인(요청 시각이 그 순간 TF 버퍼의 최신 시각보다 미래)은 동일하게
확인됨 — 라이브 `/tf` 구독은 point cloud 콜백이 도착한 시점까지 아직
재생되지 않은 미래의 TF를 절대 가질 수 없으므로, 버킷이 늦을수록(스캔
후반부일수록) 실패 확률이 커지는 것은 구조적으로 당연하다(맨 끝 버킷
0/1이 전무한 것은 5x 재생이라는 진단 조건의 타이밍 특성으로 보이며,
근본 원인 해석에는 영향 없음 — 아래 2번 수정으로 애초에 이 문제 자체가
사라지므로 정밀 재현은 생략).

### 2. 수정 — bag 전체 TF 사전 로드
`src/agconav_drone/agconav_drone/drone_elevation_mapper.py`에
`deskew_tf_preload_bag_path` 파라미터(기본 빈 문자열=꺼짐, 기존 동작
불변) 추가: 값이 있으면 노드 시작 시(`__init__`, spin 전) `rosbag2_py`로
그 bag의 `/tf`+`/tf_static`만 필터링해 전부 읽어
`self._tf_buffer.set_transform()`/`set_transform_static()`으로 채운다.
`tf2_ros.Buffer`의 `cache_time` 기본(10초)으로는 bag 전체(2258.8s)를
미리 채워도 앞부분이 금방 밀려나므로, preload를 쓸 때만
`cache_time=Duration(seconds=3600.0)`로 늘림(비-preload 경로는
`cache_time=None`으로 기존 tf2 기본값 그대로 유지 — 동작 불변 확인
목적으로 `py_compile`만 하고 별도 회귀 테스트는 안 함, 코드 변경이
조건부 분기라 명백함).

`run_results/run_variant_replay.sh`에 `<bag_dir>`을 인자로 받아
`deskew_tf_preload_bag_path`를 자동으로 넘기도록 소폭 수정(변형B'
실행에서만 실질적 영향 — A/기준 경로는 이 파라미터 자체를 안 씀,
`deskew_enabled=False`면 preload 자체를 건너뛰도록 코드에서도 가드함).
preload가 bag 전체 `/tf`(수십만 건)를 다 읽느라 노드 시작이 늦어질 수
있어, bag play 시작 전 "Accumulating..." 로그(=preload 포함 `__init__`
완료 신호)가 뜰 때까지 최대 60초 대기하도록 스크립트에 추가.

### 3. 검증 (본 실행 전 스모크테스트)
같은 bag으로 짧은 구간(5x 재생, DEBUG 로그) 스모크테스트:
- preload 완료 로그: `deskew TF 사전로드 완료: bag="...", /tf 111598건,
  /tf_static 1건` — bag 전체(`ros2 bag info` 기준 `/tf` 카운트와 정확히
  일치) 로드 확인.
- 그 후 point cloud를 흘려보내며 20초 이상 관찰 — **"deskew 구간 ... TF
  조회 실패" 로그 0건** (수정 전 진단에서는 훨씬 짧은 구간에서도 1170건
  발생했음). extrapolation 에러가 완전히 사라진 것을 확인 — 근본 수정
  성공.
### 3-1. B' 1차 본 실행 — 시간초과, 재진단
1차 `run_variant_replay.sh`(deskew_enabled만) 실행이 `capture_nav_fn.py`
타임아웃(CAP_TIMEOUT=3000s=50분)으로 "missing"(결과 못 받음) 처리됨.
크래시 로그는 없었음 — `A.log`가 4번째 "no point cloud received yet"
경고(스캔 시작 직후) 이후 **47.7분 동안 아무 로그도 없다가** stall 경고가
찍힘. 처음엔 "디스큐 콜백이 멈췄다"로 의심했으나, 실제로는 아니었다:

- ptrace 권한이 없어(`strace`/`gdb` 붙이기 `Operation not permitted`)
  `faulthandler.register(SIGUSR1)`을 임시로 심은 재현 스크립트
  (`/tmp/hang_diag.py`, 소스코드 미변경 — 별도 스크립트로 노드를 직접
  띄운 것)로 정체 구간에 SIGUSR1을 보내 스택을 덤프 → 노드는
  `rclpy.spin` 안의 정상적인 `_wait_for_ready_callbacks`(새 메시지
  대기)에 있었다. **콜백이 도는 게 아니라 메시지가 안 오길 기다리는
  정상 상태** — 무한루프/데드락이 아니었음.
- `ros2 topic hz /drone/points`로 확인한 결과, **매퍼를 아예 안 띄우고
  bag만 재생해도** `/drone/points` 순간 발행률이 2~7Hz로 들쭉날쭉함
  (평균 기대치 9.9Hz=22319건/2258.8s에 훨씬 못 미침, burst 패턴 — 반면
  `/tf`는 49~50Hz로 항상 안정적). **디스큐/preload 코드와 무관하게 이
  bag(22GB, mcap summary/인덱스 없음 — "attempted to read in receive
  timestamp order with no message index" 경고, 순차 전체 스캔으로
  폴백) 자체의 재생 성능이 원래 불안정하다**는 것을 실측으로 확인.
- 결론: ExtrapolationException 버그(1~3번)는 확실히 고쳤지만, **이번
  타임아웃은 별개의 원인** — preload(bag 전체 22GB 순차 스캔, ~수십초)
  + 디스큐(스캔당 최대 12회 TF 조회)가 원래도 불안정한 이 bag의 재생
  여유를 더 깎아, 50분 타임아웃 안에 전체(22319개 스캔)를 못 끝낸
  것으로 판단(F.log에 "지형 특성 계산 완료"가 뒤늦게 찍힌 것도 파이프라인
  자체는 끝까지 진행 중이었다는 근거).

### 3-2. 추가 개선 — TF preload를 작은 pickle 캐시로
`_preload_tf_from_bag`가 매번 22GB bag을 처음부터 순차 스캔하는 비용을
줄이기 위해, 최초 1회만 bag에서 읽고 `/tf`+`/tf_static`만 추린 작은
pickle 캐시(`<bag_path>_tf_cache.pkl`, TransformStamped는 pickle
가능함을 실측 확인)를 만들어 다음 실행부터는 그것만 읽도록 수정.

### 3-3. B' 2차 실행 — 대기 로직 버그(내 실수, 진짜 hang 아니었음)
2차 실행에서 "52초 만에 완료"로 보였으나, 이는 **내 대기 스크립트의 버그**
였다: `[ -f 결과.json ]`으로만 완료를 판정했는데, 1차 실행이 남긴
"missing" 내용의 결과 파일이 이미 그 경로에 있었고, 2차 실행이 그 파일을
아직 안 건드린 시점에 내가 존재 여부만 확인해 즉시(잘못) "완료"로
오판함. **실제로는 2차 실행도 계속 정상 진행 중**이었음(모든 프로세스
생존 확인). mtime 기준 대기로 바꿔 재확인.

### 3-4. B' 재현 진단 — "hang"의 정체를 여러 각도로 조사
mtime 기준으로 다시 지켜보니 19.5분째 `A.log`가 12초 시점에서 전혀 안
늘어나 진짜 정체로 의심, 아래 순서로 원인을 좁혀갔다(전부 실측, 추측
아님):
1. ptrace 권한 없음(`strace`/`gdb attach` 전부 `Operation not
   permitted` — 샌드박스 제약) → `/tmp/hang_diag.py`(faulthandler로
   SIGUSR1 스택 덤프 등록, 소스코드 미변경) 재현 스크립트로 우회.
2. 첫 스택: `rclpy.spin` 안의 정상적인 `_wait_for_ready_callbacks`
   (콜백 무한루프 아님).
3. `ros2 topic hz /drone/points`로 확인 — **매퍼 없이 bag만 재생해도**
   순간 발행률이 2~7Hz로 불안정(평균 기대 9.9Hz에 못 미침, burst
   패턴). `/tf`는 항상 49~50Hz로 안정 — 디스큐/preload와 무관한 이
   bag(22GB, mcap 인덱스 없음) 고유의 재생 특성으로 잠정 결론 → 오판.
4. 실제 콜백 로직을 bag 메시지로 직접 재현(`/tmp/hang_diag2.py`,
   `DroneElevationMapper._points_callback`을 함수로 직접 호출) —
   15개 메시지 전부 4.7~10.7ms, grid_shape도 정상(177x412~429, 폭주
   없음). **콜백 로직 자체는 전혀 문제없음.**
5. 메모리/스왑 10분 추적 — 2.3~2.5GB 사용, 여유(avail) 3.4~3.6GB로
   항상 안정, 스왑도 370MB에서 고정. **메모리 압박도 원인 아님.**
6. `ros2 topic info -v /drone/points` — publisher(rosbag2_player)
   1개, subscriber(drone_elevation_mapper) 1개, RELIABLE
   publisher+BEST_EFFORT subscriber로 QoS 호환. **discovery도 정상.**
7. faulthandler로 5회 연속(2초 간격) 스택 확인 — 4회는
   `_wait_for_ready_callbacks`, **1회는 `_make_handler`**(막 도착한
   콜백을 처리하려는 순간)를 잡음 → 완전히 죽은 게 아니라 이벤트가
   드물게라도 들어오고 있다는 뜻.
8. QoS를 임시로 BEST_EFFORT→RELIABLE로 바꿔 재현 — **효과 없음**(여전히
   정체 패턴), 원복.
9. **결정적 재확인**: `_check_data_received`(1Hz 체크 타이머) 코드를
   다시 읽어보니, `elapsed > data_timeout_sec(기본 2.0s)`일 때만
   경고를 찍고 **그 이하면 아무 로그도 안 남기는 설계**였다. 즉
   "no point cloud received yet"이 4번 뒤로 안 뜨는 것은 "타이머가
   멎었다"는 증거가 아니라 **"point cloud를 계속 정상 수신 중이라
   찍을 게 없다"는 뜻일 수 있다** — 지금까지의 "hang" 진단 전제 자체가
   틀렸을 가능성. 게다가 `drone_elevation_mapper`는
   `/drone/path_status`(bag 맨 끝에 딱 1건)를 받아야만 지도를 1회
   발행하는 설계라, bag 재생이 (3번에서 실측한 burst 패턴 때문에)
   rate=1.0인데도 예상(37.6분)보다 몇 배 느려지면 그만큼 늦게 끝나는
   것이지 실제 hang이 아닐 수 있다.

**결론(잠정)**: 진짜 데드락/무한루프라는 증거는 끝내 못 찾았고(콜백은
빠름, discovery 정상, 메모리 정상, QoS 무관), 오히려 "정상이지만
느리다"는 가설과 부합하는 정황(로그 설계, 1회성 발행 설계, burst
재생)이 더 많다. **CAP_TIMEOUT을 3000→6000→10800s(3시간)로 늘려 마지막
검증 실행 중**(`run_results/logs/variantBprime_run.log`,
`/tmp/trav_fn_variantBprime_deskew_fixed/`). 이번에도 3시간 안에
`bag play 종료` 로그가 안 뜨면(=play.log에 그 문구가 없으면) 그때는
정말 재생 자체가 비정상적으로(3시간 이상) 느려지는 것이니 별도 원인을
더 파야 한다.

### 진행 중 체크인 (세션 시간 예산 소진 임박)
이번 세션 타임아웃(2시간)을 이미 초과했거나 임박한 시점에서, 위 검증
실행을 백그라운드에 걸어두고 문서화로 전환한다(사용자가 실시간으로
응답할 수 없는 세션이라 "돌려놓고 다음 세션이 확인" 전략).
**다음 세션이 이어받을 때**:
1. `pgrep -af "drone_elevation_mapper|ros2 bag play"`로 위 실행이 아직
   살아있는지 확인.
2. 살아있으면: `cat run_results/variantBprime_deskew_fixed_fn.json`과
   `tail /tmp/trav_fn_variantBprime_deskew_fixed/play.log`로 진행
   상황(파일 mtime, "bag play 종료" 문구 유무) 확인 후 계속 대기하거나,
   너무 오래(예: 4시간+) 걸리면 진짜 원인 재조사 필요.
3. 죽어있고 결과 JSON이 정상(에러/missing 아님)이면 5번(비교표)·6번
   (저장/커밋/푸시)으로 바로 진행.
4. 죽어있고 여전히 missing/에러면, 위 1~9번 조사를 이어받아 실제
   `_grow_to_fit`(그리드 확장 방어)이나 `_kalman_update_cells`(이노베이션
   게이팅) 경로에서 디스큐 특유의 입력(여러 버킷에서 온, 서로 다른
   sensor_origin 근사를 쓰는 점들)이 극단적인 케이스를 만드는지 남은
   가설로 확인.
- 진단용 임시 파일(`/tmp/hang_diag*.py`, `/tmp/tf_perf_test*.py`)은
  `/tmp`라 세션 종료 시 자동 정리 대상 — 재현 필요하면 이 문서의 방법
  설명을 참고해 다시 작성.

### 3-5. B' 4차(최종) 실행 — 성공, "hang"은 진짜 버그가 아니었음
CAP_TIMEOUT 10800s(3시간)로 건 실행이 **35.2분 만에 정상 완료**됨(bag
play 시작~마지막 로그 기준, 기대치 37.6분과 거의 일치 — 오히려 예상보다
빠름). extrapolation 실패 0건, 크래시/에러 0건. **이걸로 3-4절의
"hang처럼 보였던 문제"가 진짜 코드 버그가 아니라, 그 시점의 일시적
시스템 상태(직전까지 반복한 진단 재현들 — hang_diag 스크립트들,
tf_perf_test들, 여러 번의 bag play/kill — 이 디스크 캐시를 어지럽히고
동시 프로세스 경합을 만든 것으로 추정)에 의한 것이었다는 가설이
확정됐다.** 즉 ④ 디스큐 버그 수정 자체는 처음부터 올바르게 끝나 있었고,
그 이후의 "재현 안 됨/hang" 소동은 진단 과정 자체가 만든 잡음이었던
것으로 보인다(교훈: 반복 진단 세션 사이에 시스템이 완전히 안정된
상태인지 확인 없이 곧바로 "본 실행"을 판단하면 안 됨).

## 4단계 — ④ 단독(수정본) 채점 결과

| 항목 | B(구, 버그있음) | B'(신, 버그수정) | 기준(#5) |
|---|---|---|---|
| coverage_percent | 92.95% | **94.69%** | 95.80% |
| unknown_percent(nav_map) | 9.47% | **1.35%** | 1.46% |
| wheel FN% | 15.00% | **20.51%** | 10.91% |
| leg FN% | 7.53% | **13.70%** | 8.03% |

**커버리지는 확실히 회복됐다**(unknown 9.47%→1.35%, 기준(1.46%)과 거의
동일한 수준 — extrapolation으로 버려지던 스캔 뒷부분 데이터가 이제
정상적으로 지도에 반영됨). 하지만 **FN%는 오히려 크게 악화됐다**
(wheel +5.51pp, leg +6.17pp, B 대비). 즉 버그 수정이 "더 많이
측정하게" 만들었지만, 그 새로 채워진 영역의 상당수가 GT 기준
"막힘"으로 오판됐다는 뜻이다.

**해석**: 이전(B, 버그있음) 버전이 우연히 좋아 보였던 이유는, 바로 그
extrapolation 실패로 버려지던 버킷들(스캔 중후반부, 방위각 상 특정
구간)의 점들이 원래 노이즈가 크거나 디스큐 근사(12버킷 column-index
기반 시간 근사) 오차가 가장 큰 영역이었을 가능성이 높다 — 그 데이터가
통째로 버려지면서 "노이즈가 적은 지도"처럼 보였을 뿐, 실제로는 그만큼
정보가 누락된 것이었다(unknown 9.47%가 그 증거). 버그를 고쳐 그
데이터까지 포함시키자, 그 안에 있던 원래의 노이즈/근사 오차가 그대로
지도 품질에 반영되어 FN%가 나빠진 것으로 판단된다.

## 5단계 — 최종 비교표

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% | 비고 |
|---|---|---|---|---|---|---|---|---|
| 5 | 5m AGL | 5m/s | 4m | GT만 | 칼만필터(이상치방어) | **10.91%** | **8.03%** | 기준 |
| B(구) | 5m AGL | 5m/s | 4m | GT만 | +④디스큐(버그있음) | 15.00% | 7.53% | 커버리지 손실(unknown 9.47%) — 참고용, 폐기 |
| B'(신) | 5m AGL | 5m/s | 4m | GT만 | +④디스큐(버그수정) | **20.51%** | **13.70%** | 커버리지 회복(unknown 1.35%, 기준과 거의 동일), 여전히 기준 미달 |

**결론**: 버그 수정(TF 사전로드)은 목표(커버리지 회복)를 정확히
달성했다 — B'의 unknown_percent(1.35%)는 기준(1.46%)과 사실상 같은
수준이다. 하지만 이는 ④(디스큐) 자체가 baseline보다 나은 지도를
만든다는 뜻은 아니었다: **커버리지가 회복되며 드러난 진짜 결과는, ④의
12버킷 근사가 만드는 점들이 기존(스캔 전체를 단일 TF로 처리) 방식보다
오히려 노이즈가 크거나 부정확하다는 것**이다. 직전 세션(A/B/C 실험)
결론과 종합하면, 이번 실험 조건에서는 **① 단독, ④ 단독(버그있는 버전),
④ 단독(버그수정본), ①+④ 전부 기준보다 나쁘다** — ④의 근본적인 접근
(column index 기반 시간 근사, 12버킷 분할)이 이 시뮬레이터/이 비행
조건에서는 기대한 효과(모션 블러 감소)보다 근사 오차의 악영향이 더 큰
것으로 보이며, 단순히 버그(TF 유실)를 고치는 것만으로는 baseline을
넘어서지 못했다.

**전체 타임아웃: 4.5시간(사용자가 3시간→4.5시간으로 변경 지시, 세션 중 반영).**
판단이 필요한 지점은 확인받지 않고 안전한 기본값으로 스스로 진행, 근거는 이
문서에 계속 기록한다(신규 항목은 관례대로 파일 맨 위에 추가). 범위:
`test_main_brian` 브랜치만, `git push --force` 금지, 범위 밖 소스코드 수정 금지.

### 0. 브랜치 확인
- `~/AG-CoNav-test_main`는 이번 세션 시작 시 `test_main` 브랜치였음(작업폴더가
  최근 다른 용도로 쓰인 흔적: `glim_config/config.json`,
  `src/agconav_test_worlds/launch/experiment.launch.py` 수정 + `path_100x100_5m_
  {2,3,4}m.yaml`/`run_results/` untracked). `git stash push -u`로 전부 보관(안
  버림 — `test_main` 브랜치 컨텍스트의 스태시로 남아있음, 이번 작업 범위 밖이라
  그대로 둠).
- `git checkout test_main_brian` 완료. 이 브랜치는 이미 `1d7a3aa [feat] 84m +
  방법B 실험 완료`까지 커밋되어 있고 `run_results/`도 이미 커밋된 상태(84m 실험
  산출물 전부 포함, 아래 706줄 기존 기록 그대로 보존). `git pull origin
  test_main_brian` → "Already up to date."
- 과거 기록 확인 결과 **5m AGL 실험(attempt1~4)은 전부 무효/실패**했고(climb-rate
  발산, VM 크래시, TF 동결, 정지판정 버그), 이후 목적을 84m+방법B로 전환해
  그것만 완료함. 즉 **5m AGL 스트립간격 4m(어떤 속도로도) 방법B 결과는 이번이
  최초**다 — 8절 비교표의 "5m+방법B(간격5m)" 행은 인용할 이전 데이터가 없다
  (간격 2/3/4m 경로 파일 자체는 2단계에서 만들어졌으나 velocity 비행이 전부
  실패해 방법B를 적용해본 적이 없음). 이번엔 간격4m·**속도5m/s**(기존
  attempt들은 4m/s)로 새 경로를 만들어 처음부터 시도한다.

### 1. 디스크 정리
`du -sh ~/AG-CoNav-test_main/bags/*` 결과: **디렉토리가 비어있음(파일 0개)** —
이전 세션 종료 시점에 이미 전부 정리되어 있었다(SUMMARY.md/PROGRESS.md에 attempt3
41GB, attempt4 1.6GB, 84m bag 5.1GB 모두 분석 후 삭제 기록 확인). `run_results/logs`,
`run_results/clouds`도 존재하지 않음(gitignore 대상, 이전 세션이 정리 후 커밋).

`df -h ~`: `29G/78G(39%), 여유 46G` — **이미 목표(60% 이하)를 크게 만족**한다.
지울 것이 없으므로 삭제 작업 없이 다음 단계로 진행. 보호 대상
(`path_100x100_5m_*.yaml`, `run_results/*.md,*.png`, GLIM apt 패키지, `.git`)은
확인만 하고 손대지 않음.

### 2. 사전 준비
- `pgrep -af "ros2|gz sim|docker.*glim"` → 결과 없음, 잔여 프로세스 없이 깨끗한
  상태 확인.
- 디스크 감시 스크립트 `-k` 버그: `run_results/run_attempt4_monitored.sh`,
  `run_84m_velocity_monitored.sh` 둘 다 이미 `timeout -k 5 3 ros2 topic echo ...`
  로 수정되어 있고, 84m 실험 전체를 통해 실전 검증까지 끝난 상태(SUMMARY.md
  "알려진 버그" 절 — 이 스크립트가 실제로 정상 감지/종료했음). 코드 레벨 확인
  완료로 간주.
- `colcon build --symlink-install` 성공 (`Summary: 11 packages finished [3.86s]`,
  에러 0건, deprecation 경고만 있음 — symlink-install이라 원래도 빨랐음).
- `timeout -k` 독립 재검증: SIGTERM을 무시하도록 만든 테스트 스크립트(`trap ''
  TERM; sleep 30`)에 `timeout -k 5 3 ...`을 걸어 실행 → 정확히 8초(3초 SIGTERM
  대기 + 5초 유예 후 SIGKILL)만에 `Killed`로 강제종료 확인. 코드+실전(84m
  실험)+이번 독립시험까지 3중으로 확인 완료.

### 3. 경로 재생성 (5m AGL, 방법B, 스트립간격 4m, 속도 5m/s)
`run_results/generate_path_4m_5mps.py`(신규, `generate_paths.py`의 지표면모델/
안전검증/상승률제한 로직 그대로 재사용, `SPEED_MPS=5.0`만 교체) 실행 결과:

- 웨이포인트 5,251개(원시=최종, **자동보정 0회** — 4m/s판(9회 보정, 최소클리어런스
  3.033m)보다 오히려 더 여유있게 나옴, 이유: 속도가 5m/s로 빨라지면서 같은
  climb_speed_mps 예산(6.0x0.7=4.2m/s)에서 허용 기울기 `MAX_SLOPE=4.2/5.0=0.84`가
  4m/s판(`4.2/4.0=1.05`)보다 더 완만해져 상승 프로파일이 건물 진입 전부터 더
  일찍·더 넓게 퍼져 시작됨).
- **웨이포인트 자체**: `z = 지표면(x,y) + 5.0m` 이상(rate-limited majorant라 실제로는
  이보다 더 높을 수 있음, 항상 위로만 완화).
- **웨이포인트 사이 구간(0.2m 샘플)**: 최소 클리어런스 **4.720m** ≥ 3.0m 마진 충족.
- **climb rate**: 요구 수직속도 최대 **4.20m/s** = 설계 상한과 정확히 일치(레이트
  리밋이 의도대로 작동), 예산(6.0m/s) 초과 세그먼트 **0개**.
- 경로길이 2847.4m, 등속 5m/s 기준 예상 순수비행시간 569.5s(9.5분), 턴 25회.
- 저장: `src/agconav_test_worlds/config/path_100x100_5m_4m_5mps.yaml`,
  `run_results/path_5m_4m_5mps_safety_report.md`.

### 4. 순간이동 드라이런 검증
1차 시도(`bags/dryrun_teleport_4m_5mps`)는 `drone_path_player`가
`FileNotFoundError`로 즉시 죽음 — **경로 yaml을 3단계에서 소스트리에만
저장하고 colcon build를 다시 안 돌려서** `install/`에 새 파일이 없었던 것
(symlink-install도 "새로 추가된 파일"은 재빌드해야 심볼릭링크가 생김, 기존
파일 내용 변경과는 다름). `colcon build --symlink-install --packages-select
agconav_test_worlds`로 해결 확인 후 재실행.

2차 시도(재실행) 결과: **1097초(18.3분) 만에 `path_status=True` 정상 완주.**
`run_results/analyze_bag.py`로 분석(`run_results/dryrun_4m_5mps.md`):
- **박스 기준 커버리지 99.6%**(996,488/1,000,000) — 4m/s판 드라이런(99.6%)과
  거의 동일, 5m/s로 속도만 바뀐 것이 커버리지에 미치는 영향은 무시할 수준.
- 박스 밖 유효 셀 2.92%(29,957개) — 경계 5m 이내 87.9%, 4m/s판(88.2%)과 같은
  패턴(스와스 폭 특성 + 코너 yaw반전 보간 불안정, 기존 문서화된 무해한 현상,
  `PROGRESS.md` 하단 "박스 밖 유효 셀의 공간 분포" 절 참조) — 새로 생긴 문제
  아님.
- 이상치(자기반사 의심) 0건, 고도 범위 0.936~8.958m로 지표면 모델 기대범위
  (1.202~8.780m)와 사실상 일치.
- **판정: 경로 자체는 안전/정상, 5단계(velocity 실비행) 진행.** bag(11GB)은
  분석 완료 후 삭제(재현 가능, 디스크 53%→39%로 원복).

### 5. Velocity 비행 실행
`run_results/run_velocity_4m_5mps_monitored.sh`(신규, 84m 방법B 실험 스크립트
패턴 재사용) 작성:
- `flight:=velocity path_file:=path_100x100_5m_4m_5mps.yaml cruise_speed_mps:=5.0
  module_a:=false module_f:=false` — 84m 방법B와 동일하게 모듈 A/F는 라이브로
  안 돌림(방법B는 bag만 있으면 오프라인 처리 가능 + attempt3에서 유력했던
  "module_a까지 같이 돌 때의 자원경합" 원인을 애초에 피하는 선택).
- bag: `/drone/points /drone/imu /tf /tf_static /drone/path_status`만 실질적으로
  채워짐(나머지는 launch가 항상 구독하는 고정 토픽 목록에 있지만 발행자가 없어
  bag에 안 찍힘 — 지시된 5개 토픽과 결과적으로 동일).
- 재발방지: `timeout -k` 적용, **마지막 웨이포인트(N/N) 도달 후에는 진행률
  정지를 STALL로 오판하지 않도록 예외처리**(SUMMARY.md에 기록된 attempt4의
  기존 버그 재발 방지), **module_a를 안 돌려 "TF lookup failed" 로그가 없으므로
  `ros2 topic hz /tf --window 20`를 60초 주기마다 4초 창으로 직접 찔러 /tf
  발행 여부를 독립적으로 확인**(3회=3분 연속 무응답 시 TF_FROZEN으로 즉시 중단
  — attempt3의 핵심 위험이었던 TF 동결을 이번엔 module_a 로그에 의존하지 않고
  직접 감지).
- TIMEOUT=3600s(60분, 등속 순수비행시간 569.5s의 6.3배 여유 — attempt4 선례의
  5.7배 여유율과 비슷한 수준으로 판단해 설정), DISK_LIMIT=80%, STALL_CHECKS=5분.
- 정지조건 2번(3~4회 재시도 후에도 불안정/충돌 시 중단) 적용 예정: 아래에
  attempt별 결과를 계속 기록.

**attempt1 — 1회 만에 정상 완주, 재시도 불필요.**
- 2157초(36.0분) 만에 `path_status=true`, 5251/5251 웨이포인트 전부 도달.
- `/tf` 무응답 감지가 총 2회 있었으나 둘 다 **1회(1분)만 반짝하고 바로 회복**
  (3회 연속 기준에 못 미침 — attempt3의 "40분 동결"과는 전혀 다른, 정상적인
  샘플링 노이즈로 판단). TF 동결 재발 없음.
- 디스크: 시작 39%→종료 시점 68%(24GB 여유), 80% 임계치 근처 간 적 없음 —
  module_a/f를 라이브로 안 돌린 선택이 attempt3의 근본원인(추정: I/O 경합)을
  효과적으로 피한 것으로 보임.
- SIGINT로 launch가 30초 안에 안 죽어 SIGTERM으로 에스컬레이션됨(정상 동작
  범위 — shutdown_launch 로직이 설계한 대로 작동) → 그 여파로 `ros2 bag
  record`가 metadata.yaml을 못 쓰고 죽어 **bag 메타데이터 유실**(84m 실험 때와
  동일 패턴, SUMMARY.md "알려진 버그"와 같은 계열). `ros2 bag reindex -s mcap
  bags/velocity_4m_5mps_attempt1`로 완전 복구 확인: **21,771 스캔, TF
  109,013개, path_status 1개(True), 21.4GiB, duration 2183.2s** — 재비행
  불필요.
- 결론: **정지조건 2번(3~4회 재시도) 발동 없이 1회차에 성공**, 6단계로 진행.

### 6. 방법B(GT pose + GICP 정합) 적용

**시도1(원본 스크립트, 84m 실험에서 그대로 재사용)이 OOM-kill됨.** 84m 실험은
스캔 ~1,670개였지만 이번 5m AGL 4m/5mps 비행은 스캔 21,771개(13배)라, 원본
`build_methodB_cloud.py`가 전체 비행 분량의 점을 파이썬 리스트(baseline_chunks,
methodb_chunks)에 다 들고 있다가 마지막에 한 번에 `np.concatenate`하는 방식이
이 VM(5.8GB RAM)에서 감당이 안 됐다. 스캔 21750/21771까지(99.9%) 정상 처리하고
마지막 concatenate 직전에 죽음 — 파이썬 예외/트레이스백 없이 그냥 사라져서
처음엔 원인이 불명확했으나, `/var/log/syslog`에서 확증:
```
oom-kill: ... task=python3,pid=19588 ...
Out of memory: Killed process 19588 (python3) total-vm:10044336kB, anon-rss:4695904kB
```
(참고: 이 확인 과정에서 background 작업 실행 방식도 문제가 있었음 — Bash
run_in_background으로 직접 띄운 무거운 계산이 지정한 타임아웃보다 먼저
알수없는 이유로 두 번 종료됨(각각 30분/90분 지정, 그보다 일찍 "killed"),
프로세스 자체의 이슈인지 tool 실행환경 이슈인지 불명확해 이후로는
`nohup ... &; disown`으로 완전히 분리한 프로세스를 띄우고 별도 폴링으로
감시하는 방식으로 전환함 — 재현되면 다음 세션도 이 방식을 쓰는 게 안전.)

**조치**: `build_methodB_cloud.py`를 스트리밍 방식으로 재작성(판단: 방법론
자체나 실험 범위를 안 건드리고 순수 구현 최적화이므로 확정 범위 밖 변경
아님, 정지조건 3번 해당 안 됨) —
- 스캔마다 바로 `.raw`(헤더 없는 연속 float32) 파일에 append, 파이썬 리스트에
  전체 이력을 안 쌓음(GICP 타겟용 `window`는 원래도 최근 6스캔만 유지해 작음).
- 끝에서 `.raw` → `.npy` 변환도 `np.lib.format.open_memmap`으로 만든
  디스크백킹 배열에 2M점씩 청크로 복사(전체를 한 번에 메모리에 안 올림).
- **재검증**: RSS를 스캔 750개 시점에 직접 확인(`ps`) → **116MB**(원본 방식이면
  이 시점 이미 수백MB~1GB대로 자라고 있었을 것) — 픽스가 의도대로 작동함을
  확인.
- 부수 조치: `feed_cloud.py`(7단계에서 이 cloud를 Module A/F에 흘려보낼 때 씀)도
  같은 OOM 위험이 있어(`np.load`로 전체를 한 번에 RAM에 올림) `mmap_mode='r'`
  + `astype(..., copy=False)`로 지연로딩되게 최소 수정(범위: 이 실험 파이프라인이
  이 VM에서 끝까지 돌아가는 데 필수적인 인프라 수정으로 판단, 방법론/토폴로지
  변경 아님).

시도2(수정판) 결과: **성공.** RSS를 스캔 750개 시점 확인 시 116MB로 안정
(원본이면 이 시점 이미 커지고 있었을 값) — 전체 21,771 스캔 완주:
```
스캔 21771개 (TF 조회 실패로 스킵 0개)
GICP: 성공 21398, 실패(GT로 대체) 372, 첫 스캔이라 스킵 1 (성공률 98.3%)
baseline 점 266,757,021개 -> run_results/clouds/baseline_4m_5mps.npy (3.0GB)
방법B    점 266,757,021개 -> run_results/clouds/methodb_4m_5mps.npy (3.0GB)
```
(참고: 84m 실험 대비 GICP 성공률 98.3% vs 84m의 94.5% — 5m AGL의 조밀한
근거리 스캔이 GICP 수렴에 더 유리했던 것으로 보임.)

두 cloud 확보 후 원본 flight bag(22GB, 디스크 76%까지 올라간 상태)은 분석
불필요해져 삭제(재현 가능 — 재비행 스크립트로 언제든 재현) → 디스크
47%로 원복.

### 7. Wheel FN% 채점
`run_results/run_traversability_fn_v2.sh`(원본 `run_traversability_fn.sh`과
동일 로직, `capture_nav_fn.py` 타임아웃만 180s→1500s로 확대 — 이번 cloud가
84m 대비 훨씬 커서 `feed_cloud.py`가 200,000점/0.5s로 전부 흘리는 데만
~4~5분 이상 걸림, 원본 180s로는 시간 안에 못 끝남) 재사용.

**baseline_4m_5mps (정합 없음, GT pose만) 결과**:
- wheel FN% = **13.44%** (618,378개 GT통과가능 셀 중 83,102개 막힘오판,
  미측정 2.35%)
- leg FN% = **12.60%** (77,921개 막힘오판, 미측정 2.35%)
- elevation_map 커버리지 86.4%(997,553/1,154,520셀 — 그리드가 100x100보다
  약간 큼(108.0x106.9m), `_grow_to_fit` 기존 결함 계열, 2~4단계에서 이미
  무해함을 확인한 것과 같은 패턴)

**방법B(GICP) 1차 채점 — 파국적 결과, 원인 진단 후 재작업.**

1차 결과(`methodb_4m_5mps_fn_UNCORRECTED_gicp_outlier.json`로 보존):
wheel FN% **46.49%**, leg FN% **34.61%** — baseline보다 오히려 훨씬 나쁨.
elevation_map이 108x106.9m(baseline)에서 **128.2 x 388.5m**로 비정상 팽창,
`height_max=84.006m`(이 실험의 지표면 기대범위 1.2~8.9m를 완전히 벗어남 — 참고로
"84"라는 숫자가 이 리포의 다른 실험(84m 고도)과 우연히 겹쳐서 처음엔 혼동
가능성이 있었으나 무관한 우연의 일치임, 실제로 GICP가 잘못 수렴한 절대값일 뿐).

**직접 원인 확인**: `methodb_4m_5mps.npy`(2.67억 점)를 직접 스캔 → z>15m 또는
|y|>250m인 명백한 이상치 **44,864개(0.017%)** 존재, 최대 z=84.09m, 최대
y=218.68m. 파이프라인/채점 스크립트 버그가 아니라 **cloud 자체에 실제로
박혀있는 오염**임을 확인(방법B 생성 스크립트 자체 문제, Module A의 기존
`_grow_to_fit` 무제한 성장 결함이 이 오염을 극단적으로 증폭시켜 그리드
전체와 FN%를 다 망가뜨림).

**근본원인 추정**: `build_methodB_cloud.py`의 GICP 호출이 `result.converged
== True`만으로 결과를 신뢰하는데, 이번 경로(코너 25회, 84m 기준선의 4회보다
훨씬 많음)의 코너(180도 yaw 반전)마다 스캔이 극도로 작아지는 구간이 반복되고
(빌드 로그에 "point cloud is too small(2~10점)" 경고 다수 관측), 이런
저정보 상황에서 GICP가 물리적으로 말이 안 되는 국소해로 "수렴"할 수 있음을
실측으로 확인. 이건 84m 실험(코너 4회뿐)에선 거의 안 보였던(원시 통계만
소폭 악화, p95 1.62→2.66m) 현상이 코너 수가 훨씬 많은 이번 5m AGL 경로
에서는 파국적 규모로 증폭된 것으로 판단.

**조치(판단, 확정범위 밖 아님으로 결론)**: `build_methodB_cloud.py`에
"GT 대비 GICP 이동량이 물리적으로 타당한 범위(2.0m, GICP
max_correspondence_distance=1.0m 탐색폭 감안 시 정상 보정량은 원래 수십cm대
여야 함)를 벗어나면 신뢰 안 하고 GT로 폴백"을 추가 — 원래 코드에 이미 있던
"GICP 실패 시 GT 안전 폴백" 설계(`result.converged==False`만 다루던 것)를
"수렴은 했지만 말이 안 되는 해"까지 포괄하도록 완성한 것으로, 방법론/실험
범위 자체를 바꾸는 게 아니라 스크립트의 기존 안전장치를 의도대로 완성하는
버그 수정에 해당한다고 판단(정지조건 3번 미해당). 1차(미보정) 결과는 진단
가치가 있어 파일명에 `UNCORRECTED_gicp_outlier`로 명시해 보존.

**뼈아픈 실수**: 방법B cloud 생성 직후 "재현 가능하다"고 판단해 원본
flight bag(22GB)을 이미 삭제해버려서, 이 수정을 반영하려면 **재비행이
필요**했다(attempt2, 아래 계속 기록). baseline cloud는 이 결함과 무관(GT
pose만 쓰므로 GICP 자체가 안 들어감)해서 재사용 가능, methodb만 재생성.
앞으로는 방법B처럼 "정합 실패 시나리오를 다시 봐야 할 수도 있는" cloud
생성 단계에서는 채점까지 완전히 끝나기 전엔 원본 bag을 지우지 않는 게
안전하다는 교훈.

**재비행(attempt2) 및 보정판 방법B 결과**:
- attempt2: `run_velocity_4m_5mps_monitored.sh attempt2`로 재비행. TF는
  이번에도 정상(중간 1회 블립 후 즉시 회복), 그러나 **wp 5239/5251(99.8%)
  에서 디스크 80% 임계치에 정확히 도달해 설계대로 즉시 중단**(DISK_THRESHOLD_
  STOP, 정상 안전동작 — attempt1보다 총 기록량이 약간 더 많았던 것으로 보임,
  bag 24.6GiB). `path_status`는 못 찍었지만(마지막 12개 웨이포인트 도달 전
  중단) 방법B 생성은 `/tf`+`/drone/points`만 있으면 되므로 이 bag도 완전히
  유효 — 재비행은 불필요, 이 bag으로 바로 진행. bag도 SIGTERM 에스컬레이션
  여파로 메타데이터 유실 → `ros2 bag reindex -s mcap`으로 복구(25,117 스캔,
  TF 125,584개).
- 중단 시점 디스크가 81%까지 순간적으로 넘어감(정지조건 "80% 초과시 즉시
  중단"은 지켰으나 감시주기가 60초라 마지막 한 틱 사이에 79%→81%로 건너뜀) —
  즉시 진단 완료 후 안 쓰는 파일(1차 미보정 cloud 3GB) 삭제로 77%로 낮추고
  계속 진행, 이후 방법B cloud 생성 완료 직후 attempt2 bag(25GB)도 삭제해
  48%로 안정화.
- **보정판(이동량 상한 2.0m 필터 적용) GICP**: 성공 24,707, 실패(GT대체)
  409(그 중 "수렴했지만 이동량 상한 초과로 기각" 49개 — 새로 추가한 안전장치가
  실제로 작동함을 확인), 점 309,125,893개.
- **오염 재검증**: cloud 직접 스캔 결과 z>15m 또는 |x|>100 또는 |y|>250
  이상치 **996개/3.09억(0.0003%)**로 격감(1차 미보정판 44,864개 대비 98%
  감소). z 범위 0.068~15.09m로 지표면 기대범위(1.2~8.9m)에 훨씬 가까워짐
  (완전히 0인 건 아니라 100% 클린은 아니지만, 그리드를 파국적으로 부풀리던
  수준은 해소).
- **보정판 채점 결과**: elevation_map 크기 109.7x106.2m(baseline 108x106.9m과
  비슷한 정상 범위로 복귀, 1차의 128.2x388.5m 폭주 해소), 커버리지 89.2%.
  **wheel FN% = 37.26%, leg FN% = 30.03%** — 1차 파국적 결과(46.49%/34.61%)
  보다는 크게 개선됐지만, **baseline(13.44%/12.60%)보다는 여전히 훨씬
  나쁘다.** step 중앙값도 baseline 0.0092m → 방법B 0.0489m(5.3배), wheel
  기준(0.08m) 초과 비율도 16.1%→38.8%로 뚜렷이 악화 — 이제는 이상치 몇 개가
  아니라 **전반적인 노이즈 증가**로 보인다.
- **해석**: 84m 실험(코너 4회)은 GICP가 wheel FN%를 개선했지만, 이번 5m AGL
  4m/5mps 경로(코너 25회, 스트립마다 180도 yaw 반전)는 코너마다 스캔이
  작아지는 구간이 잦아 GICP가 안정적으로 정합할 타겟/소스 형상 자체가
  부족한 경우가 훨씬 많이 반복된 것으로 보인다(이동량 상한으로 최악의
  경우는 걸렀지만, 상한 안에 드는 "그럴듯하지만 부정확한" 미세 오정합까지는
  못 거름 — 이게 누적되며 baseline보다 못한 결과를 만든 것으로 판단). 즉
  **이번 실험 조건에서는 방법B가 순수하게 역효과였다** — 84m과 정반대 방향
  결과.
- 산출물: `methodb_4m_5mps_fn.json`(보정판, 최종), `methodb_4m_5mps_fn_
  UNCORRECTED_gicp_outlier.json`(1차 진단용 보존), `run_results/clouds/
  {baseline,methodb}_4m_5mps.npy`.

### 8. 비교표

전체 비교표와 분석은 `run_results/SUMMARY.md` 맨 위 절("5m AGL 방법B 실험")에
정리했다. 핵심 결론:
- **간격(5m→4m) 자체의 개선 효과는 이번 실험만으로 확인 불가** — 5m 간격
  방법B 결과가 존재한 적이 없음(이전 세션 5m AGL 시도 전부 실패).
- **고도 자체(84m→5m AGL)는 GT 기준선 기준 뚜렷한 개선**: wheel FN%
  23.94%→13.44%(-44% 상대), leg는 거의 동일(12.15%→12.60%).
- **정합(GICP) 효과는 84m과 5m AGL/4m/5mps에서 정반대**: 84m은 개선
  (wheel -16% 상대), 이번 5m AGL/4m간격/5m/s는 **악화**(wheel +177% 상대,
  leg +138% 상대) — 코너가 훨씬 잦은(25회 vs 4회) 저고도 lawnmower 경로에서
  코너마다 반복되는 저정보(작은 점군) 스캔 구간이 GICP를 체계적으로
  불안정하게 만든 것으로 판단(자세한 원인 진단은 7절 참조).
- **실용적 결론**: 이번 실험 조건(5m AGL, 4m 간격, 5m/s, 코너 잦은 lawnmower)
  에서는 방법B(GICP 정합)를 쓰지 않고 GT pose만 쓰는 쪽(baseline)이 명백히
  더 낫다. 방법B는 "코너가 드문 경로"라는 조건에서만 순이득이었을 가능성이
  높다.

### 9. 저장/보고

**산출물 최종 목록**:
- `src/agconav_test_worlds/config/path_100x100_5m_4m_5mps.yaml` — 이번 실험
  경로(5,251 웨이포인트, 5m/s, 4m 간격, 5m AGL).
- `run_results/path_5m_4m_5mps_safety_report.md` — 경로 안전검증 보고서.
- `run_results/generate_path_4m_5mps.py` — 경로 생성 스크립트(재사용 가능).
- `run_results/run_velocity_4m_5mps_monitored.sh` — velocity 비행 실행+감시
  스크립트(재발방지 반영: `timeout -k`, 마지막 웨이포인트 STALL 오판 방지,
  `/tf` 직접 감시).
- `run_results/build_methodB_cloud.py` — 방법B cloud 생성(이번 세션에 두
  가지 버그 수정: OOM 방지 스트리밍 방식 재작성, GICP 이동량 상한 안전장치
  추가 — 84m 실험에서도 재사용 가능한 개선).
- `run_results/run_traversability_fn_v2.sh` — FN% 채점 스크립트(캡처
  타임아웃만 확대, 원본 로직 무수정).
- `run_results/{baseline,methodb}_4m_5mps_fn.json` — 최종 채점 결과.
- `run_results/methodb_4m_5mps_fn_UNCORRECTED_gicp_outlier.json` — 진단용
  보존(GICP 이동량 상한 적용 전, 파국적 오염 결과).
- `run_results/clouds/{baseline,methodb}_4m_5mps.npy` — 누적 point cloud
  (각 3.0~3.7GB, git 미포함 — 84m 실험과 동일 관례로 재현 가능해서 로컬만
  보존, bag은 이미 삭제).
- `run_results/SUMMARY.md` 맨 위 절, `run_results/PROGRESS.md`(이 문서) —
  전체 과정/판단근거 기록.
- 부수 수정: `src/agconav_test_worlds/scripts/feed_cloud.py`(mmap_mode='r'로
  OOM 방지, 큰 cloud를 이후에도 안전하게 피드할 수 있게).

**디스크 최종 상태**: 48%(35G/78G 사용, 39G 여유) — 시작 시(39%)보다 약간
높지만 목표(60% 이하)를 여유있게 만족. bag은 전부 삭제, cloud만 로컬 보존.

**타임아웃 준수**: 세션 시작 약 03:53, 완료 약 07:00 — 총 소요 약 3시간 10분,
사용자가 변경한 4.5시간 예산 안에서 완료.

**git 커밋/푸시 완료**: `test_main_brian` 브랜치에 커밋 `4d73d5f`
("[feat] 5m AGL + 방법B(간격4m, 5m/s) 실험 완료 — 84m과 정반대로 GICP가 FN%
악화시킴 확인"), `git push origin test_main_brian` 성공(`1d7a3aa..4d73d5f`).
25개 파일 변경(경로 yaml, 스크립트 6개, 채점 결과 3개, 보고서 2개, 로그
12개, PROGRESS/SUMMARY.md). cloud(.npy, 6.9GB)와 진단용 대용량 npz(surface_
cache.npz 650KB, dryrun_4m_5mps_grid.npz 94MB)는 84m 실험과 동일 관례로
git 미포함(재현 가능, 로컬 보존).

**최종 상태 확인**: `pgrep -af "ros2|gz sim|docker.*glim"` → ros2-daemon
표준 헬퍼만 남음(무관, 안 건드림), 실험 관련 프로세스 전부 정상 종료.
`df -h ~` → 48%(35G/78G, 39G 여유) — 시작(39%)과 비슷한 수준으로 안정.

## 세션 종료

모든 단계(0~9) 완료. 다음 세션이 이어받을 것은 없음 — 이번 실험은 완결됨.
후속 조사가 필요하다면(사용자 판단): (a) 5m AGL에서 코너를 줄인 경로(예:
스트립 진행방향을 길게, 옆이동만 최소화)로 GICP가 실제로 코너-저정보 가설이
맞는지 검증, (b) GICP 이동량 상한(2.0m)을 더 타이트하게 줄이거나 타겟
윈도우의 최소 점 개수 문턱을 추가해 "그럴듯하지만 부정확한" 미세 오정합까지
걸러내는 실험, (c) 5m 간격 방법B를 실제로 완주시켜 이번 4m 간격과 비교
(간격 자체의 효과를 마침내 분리 측정).

---

## ✅ 완료 — 84m + 방법B(GT pose + GICP 정합) 실험, 목적 정정 후 재설계

사용자가 실험 목적을 정정: GLIM 자체 odometry가 아니라 GT pose를 신뢰하고
`small_gicp`(GLIM 내부 라이브러리)로 정합만 빌려 elevation map 노이즈를
줄이는 게 목적. 5m AGL 실험(attempt1~4, 전부 크래시/정지/무효)은 이 목적에
더 이상 안 맞아 중단, 이번 84m+방법B로 새로 설계해 완료했다.

- GLIM 자체는 외부 pose 주입/정합 전용 모드를 공식 지원 안 함(GitHub #193
  미해결로 확인) → GLIM을 통째로 쓰지 않고 `small_gicp` python 바인딩을
  직접 pip 설치(`pip3 install --user --break-system-packages small_gicp`,
  arm64 wheel 있음) 후 `run_results/build_methodB_cloud.py`를 새로 작성.
- 84m 비행(`path_100x100.yaml`, 8m/s, 10웨이포인트)은 167초 만에 정상
  완주, `path_status=True` 발행. 감시스크립트의 정지(stall) 오판 버그로
  5분 뒤 불필요하게 중단됐지만 bag은 `ros2 bag reindex`로 완전 복구(재비행
  불필요) — 자세한 내용은 `run_results/SUMMARY.md` 맨 아래 "알려진 버그" 참조.
- **결과: wheel FN% 23.94%→20.10%(개선), leg FN% 12.15%→11.89%(거의 무변화),
  단 원시 단차 통계(p95 1.62→2.66m)는 악화, 미측정 비율도 증가** — 정합이
  "쉬운 곳은 더 쉽게, 어려운 곳은 더 어렵게" 만드는 트레이드오프였다.
  전체 분석/해석은 `run_results/SUMMARY.md` 참조.
- 산출물: `build_methodB_cloud.py`, `run_84m_velocity_monitored.sh`,
  `84m_baseline_fn.json`/`84m_methodB_fn.json`, `maps_84m/` — 전부 git 커밋.
  bag(5.1GB)/point cloud(166MB)는 재현 가능해 git 미포함(bag은 삭제,
  cloud는 로컬 보존).

## 🟡 전체 정지 + 디스크 정리 (사용자 지시, 재시도는 별도 지시 대기)

**1) 프로세스 정지**: attempt4(진행 중이던 velocity 비행) 및 내 감시스크립트
(`run_attempt4_monitored.sh`) 전부 SIGINT→(bag record/launch 프로세스만
SIGTERM까지 필요)로 정상 정지. gz sim/bridge/follower/mapper는 개별
SIGINT로 곧바로 종료됨. 최종 확인(`pgrep -af "ros2|gz sim|docker.*glim"`)
결과 잔여 없음(ros2-daemon 표준 헬퍼 프로세스만 남음, 실험과 무관이라
안 건드림).

**2) 정리 전 디스크 조사 결과**

| 위치 | 크기 | 비고 |
|---|---|---|
| `~/AG-CoNav-test_main` 전체 | 1.8G | |
| `bags/velocity_4m_attempt4` | 1.6G | attempt4 미완주 bag(중단됨) |
| `run_results/` | 177M | .md/.png 등 산출물 포함, 보존 대상 |
| `build/` | 23M | colcon 빌드 산출물, 보존(재빌드 방지) |
| `install/` | 2.3M | colcon 설치본, 보존 |
| `log/` | 1.1M | colcon 로그, 보존 |
| `docker system df` | **확인 불가** | permission denied — docker 그룹 권한 미해결(이전부터 계속) |
| `/var/lib/docker` 직접 확인 | **확인 불가** | 마찬가지로 권한 없음(`sudo`도 비밀번호 필요) |
| `/var/cache/apt/archives` | 198M | |
| `df -h ~` (정리 전) | 30G/78G(41%) | attempt3 bag은 이전에 이미 삭제해둔 상태라 이번 위기 수준은 아니었음 |

**docker 관련 확인 불가 — 사용자 조치 필요할 수 있음**: `docker system df`도
`/var/lib/docker` 직접 열람도 전부 permission denied(docker 그룹 미가입 +
`sudo`가 비밀번호 요구, 이전 GLIM PPA 작업 때와 동일한 세션 제약).
Docker 자체는 이제 안 쓰기로 했으니(PPA 전환 완료) 지워도 된다는 승인은
받았지만, **제가 직접 지울 방법이 없다.** 유의미한 용량이면 사용자가
직접 터미널에서 `docker system prune -a` 등을 실행해야 함 — 다만 이번
정리의 핵심 목표(bag 파일)는 이미 해결되므로 급하지 않음.

**3) 삭제 실행**: `bags/velocity_4m_attempt4`(1.6G, 미완주) 전체 삭제,
`/tmp/glim_*.log`(합계 <1KB, 이전 GLIM 스모크테스트 부산물) 삭제.
`apt clean`은 `sudo` 비밀번호 필요로 건너뜀(198M 그대로 — 영향 미미).

**4) 정리 후 디스크**: `df -h ~` → **29G/78G(39%), 여유 46GB**
(정리 전 30G/78G 41% → 큰 차이 없음, attempt4가 중단 시점에 1.6G밖에
못 쓴 상태였어서 애초에 위기 수준이 아니었음 — attempt3 삭제가 이미
끝난 뒤라 이번 정리는 예방적 성격). 보호 대상(경로 yaml 3개, run_results
내 .md/.png 전부, GLIM apt 패키지, .git)은 전부 그대로 확인됨.

## 🟢 근본원인 정정 — Module A/F 문제 아님, 순수 LiDAR 원본 데이터량 문제

사용자 질문("모듈 A,F만 돌리는데도?")에 답하며 재조사, 앞선 "리소스 부족"
가설을 더 구체화/정정함:

- **Module F는 애초에 안 돌고 있었다** — attempt3 launch 커맨드에
  `module_f:=true`를 안 넣어 기본값(false)으로 실행됨. flight log에
  `terrain_feature_calculator`/`verdictor` 관련 줄 0건으로 확인. 즉
  wheel/leg 관련 토픽엔 발행자가 아예 없어 부하 0.
- **`/drone/elevation_map`도 원인이 아니다** — `drone_elevation_mapper.py`
  소스 확인 결과 이 토픽은 **`path_status`가 true가 될 때 딱 1번만
  발행**(`self._published` 플래그로 막음). attempt3는 완주를 못 해
  `path_status`가 끝까지 false였으므로 **이 토픽은 이번 bag에 아예 한
  번도 안 찍혔다.** (그리드가 1658x2506칸까지 부푼 것 자체는 사실이고
  여전히 RESULTS.md 9절 기존 결함이지만, 이번 I/O 문제의 원인은 아님.)
- **진짜 원인 — 순수 원본 LiDAR 포인트클라우드 유량**: `model.sdf`의
  os1_lidar 스펙 `horizontal samples=1024, vertical samples=32,
  update_rate=10`Hz → 초당 327,680점. PointCloud2 포인트당 대략
  20~30바이트(xyz+intensity+ring+time)로 잡으면 **`/drone/points` 하나만
  으로 초당 약 9~10MB의 지속적 쓰기 부하**가 나온다. 66분(강제종료까지
  걸린 실제 시간) 동안 지속하면 9.5MB/s×3960s ≈ **37GB — 실측 bag 크기
  41GB와 거의 일치.** 이게 거의 전부를 설명한다.
- **"약 190~195초마다" 패턴의 정체**: `ros2 bag record`의
  `--max-bag-size 2000000000`(2GB) 분할 기준을, 9.5MB/s로 나누면
  2000MB/9.5MB/s ≈ 210초 — 관측된 190~195초 간격과 거의 일치. 즉 그
  경고는 **매 2GB 분할파일이 넘어갈 때마다 파일 마무리(flush/finalize)
  가 유입 속도를 못 따라잡아서** 뜬 것 — Module A/F 연산 부하가 아니라
  순수 디스크 쓰기 처리량 문제.

**결론(정정)**: 문제는 Module A/F의 계산량이 아니라, **이 LiDAR 스펙
(1024x32@10Hz)의 원본 데이터 자체가 이 VM의 디스크 쓰기 대역폭에 근접/
초과하는 양**이라는 것. 정상적으로 완주했을 경우(의도한 ~42분)라면
9.5MB/s×2520s ≈ 24GB 정도로, 삭제 후 확보한 여유(46GB)보다 작아 80%를
안 넘겼을 가능성이 높다 — **이번 93% 근접은 주로 TF/follower 정지로
비행이 66분까지 비정상적으로 길어진 것 + 내 감시스크립트 버그가 겹친
결과**이지, LiDAR 기록 자체가 원천적으로 감당 불가능한 수준은 아니다.
다만 여유가 넉넉하지는 않으므로 디스크 감시(80% 임계치)는 계속 유지해야
한다.

**TF 동결 원인은 여전히 미확정**이지만, sim=999s부터 이미 2~3회의 2GB
분할-지연 경고가 반복된 시점과 겹치는 건 사실 — I/O backpressure가
`exp_drone_tf_bridge`(가벼운 프로세스인데도) 스케줄링을 밀어냈을
가능성이 여전히 유력한 가설. 확정은 못 함.
## 🔴 attempt3 근본원인 조사 결과 — bag 기록 I/O가 데이터 유입을 못 따라감

**사용자 지시로 재시도 전에 원인부터 조사. 삭제(승인받음) 완료, 디스크
29G/78G(39%)로 정상화.**

`~/.ros/log/latest/launch.log`(launch 시스템 자체의 이벤트 로그, 노드별
stdout과 별개)에서 결정적 증거 발견:
```
1786789778.00  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
1786789970.55  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
1786790166.83  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
... (이하 약 190~195초 간격으로 종료 시점까지 총 20회 이상 반복)
```
**비행 시작 3.3분 지점(19:29:38)부터 강제종료 시점(20:34:26)까지 처음부터
끝까지 약 3분마다 계속 반복됐다** — `ros2 bag record`의 내부 캐시가
들어오는 메시지(특히 `/drone/points` 포인트클라우드)를 디스크에 쓰는
속도를 따라잡지 못해 계속 밀렸다는 rosbag2 자체 경고. 이게 **일회성이
아니라 처음부터 끝까지 만성적**이었다는 게 핵심.

**TF 동결과의 상관관계**: `exp_drone_tf_bridge`가 실패하기 시작한 시점
(sim 999s경, "latest data" 값이 999→1037→1043으로 서서히만 증가하며
따라잡지 못하다가 1043.000001에서 완전히 멈춤)이 이 bag 캐시 적체가
이미 5회 이상 반복된 이후였다 — **원인은 코드 결함이 아니라 이 VM
(4 core/5.8GB, 이미 RESULTS.md에도 "6-core 환경도 CPU 포화됐다"고 기록됨)의
디스크 I/O/CPU가 이만한 데이터량(포인트클라우드+지도+주행성 등 11개
토픽 동시 기록)을 감당 못 해, bag 기록이 밀리면서 다른 프로세스(특히
가벼워야 할 pose 브리지)까지 시스템 차원에서 자원 경합에 밀려 결국
멈춘 것으로 판단.** 정확한 최종 트리거(브리지 프로세스 자체의 큐
오버플로/데드락 여부)까지는 100% 확정 못 했으나, 타이밍 상관관계가
뚜렷해 "리소스 부족"을 유력한 근본원인으로 결론.

**attempt4 전 조치(사용자 확인 필요, 아직 미적용)**:
1. (필수) 감시스크립트 버그 수정: `timeout 3 ros2 topic echo` →
   `timeout -k 5 3 ros2 topic echo`로 강제종료 유예 추가 — 안전장치가
   또 먹통되는 일 방지.
2. (검토 필요) bag 기록 부하 자체를 줄이는 방법 — 예: 이번 5단계
   채점에 당장 안 쓰는 토픽(`/wheel/nav_map`,`/leg/nav_map` 등 4단계
   시점엔 아직 안 채워지는 것들)을 기록 목록에서 빼거나, 압축/청크
   크기를 조정. 다만 이건 확정 범위(정지조건 3) 밖 변경일 수 있어
   사용자 판단 필요.
3. 리소스 부족이 근본원인이라면 attempt4도 재현될 위험이 있다는 점을
   사용자에게 명시적으로 알리고 진행 여부를 확인해야 함.

## 🔴 attempt3 실패 — TF 동결 + follower 정지 + 감시스크립트 버그로 디스크 93%까지 방치

**결론부터: attempt3는 무효, 삭제 필요(사용자 확인 대기 — 디스크 위급이라 긴급).
attempt4 전에 원인부터 조사해야 함, 무작정 재시도 금지.**

**1) 감시 스크립트 자체의 버그 — 안전장치가 40분간 먹통이었다**
`run_attempt3_monitored.sh`의 `PSTATUS=$(timeout 3 ros2 topic echo
/drone/path_status --once 2>/dev/null)` 줄이 **19:55:38부터 20:35:53까지
약 40분간 멈춰 있었다.** `timeout`은 기본으로 SIGTERM만 보내는데
`ros2 topic echo`(rclpy) 프로세스가 이를 무시/처리 못 해 안 죽었고
(`-k` 강제종료 옵션을 안 넣은 게 내 실수), 이 한 줄이 루프 전체를
막아버려서 **그동안 진행률 로그도, 무엇보다 사용자가 이번에 특별히
요청한 디스크 80% 감시도 완전히 멈춰 있었다.** 이게 이번 세션의 핵심
실수 — 다음에 다시 만들 때는 `timeout -k 5 3 ...`처럼 강제종료 유예를
반드시 넣어야 한다.

**2) 그 사이 실제로 벌어진 일 — 지상 정답(TF) 동결 + 비행 정지**
- `drone_elevation_mapper`가 **sim 시각 1043.000001s(비행 시작 후 약
  17분 지점)부터 끝까지 "TF lookup failed ... extrapolation into the
  future"를 8,043회 연속으로 찍었다** — `/model/X3/pose`→`/tf` 브리지
  (`exp_drone_tf_bridge`, PID 4016)가 그 시점 이후 새 변환을 전혀
  발행하지 못한 것으로 보인다(프로세스 자체는 CPU를 계속 쓰며 살아있었
  으나 출력이 멈춘 상태 — 정확한 원인 미상, ros_gz_bridge의 장시간 Pose_V
  브리징 버그이거나 이 VM의 리소스 부족(4 core/5.8GB, load average
  2.9대)으로 인한 것일 가능성이 있으나 확정 못 함).
- 결과: **비행 시간의 96% 이상(sim 1043s~4184s+) 동안 Module A가 사실상
  아무 것도 매핑 못 함.** 완주했더라도 이 bag은 Module A 채점/GLIM 비교용
  으로 못 쓴다.
- `velocity_path_follower` 자체도 **wp 5239/5251(99.8%, 끝에서 12개
  남음)에서 진짜로 정지**했다 — 115초 넘게 위치(-30.69, -66.18, 6.72)가
  전혀 안 바뀜, 스스로 회복 못 함. (TF 동결과 같은 근본 원인일 가능성
  — follower가 쓰는 pose 소스도 결국 같은 브리지/월드 상태에 의존할 것.)

**3) 디스크 — 93%까지 방치됐다가 감시가 살아나며 정상 트리거됨, 하지만 늦었다**
감시가 멈춰있던 40분 사이 bag이 17GB→41GB로 계속 자랐고(TF/follower는
멈췄어도 `/drone/points`,`/drone/imu` 원본 스트림과 bag record는 안
멈췄음), 20:35:53에 디스크 93%를 찍자 스크립트가 (내가 수동으로 멈춰있던
`topic echo`를 강제종료해 루프를 풀어준 직후) **의도대로 즉시 감지해
`DISK_THRESHOLD_STOP`을 기록하고 SIGINT→SIGTERM 종료 절차를 실행**했다
— 로직 자체는 옳게 작동했다, 다만 40분 늦게. SIGTERM까지 줬는데도
`parameter_bridge`×4, `ros2 bag record`, `drone_elevation_mapper`가
안 죽어 **내가 수동으로 SIGKILL**해서 정리함. 정리 후 디스크
`69G/78G(94%), 여유 4.9GB`에서 더 안 늘고 안정화 확인.

**4) 판정 — attempt3 bag(41GB) 무효, 삭제 필요(긴급 확인 요청)**
경로 미완주(follower 자체 정지) + TF 동결로 Module A 데이터 96% 손실,
어느 쪽으로도 재사용 불가. 디스크가 94%(4.9GB 여유)라 이 41GB를 지워야
다음 시도든 GLIM 처리든 여유가 생긴다. `rm -rf`가 자동승인 밖이라 삭제는
사용자 확인 후 진행 — 이번엔 디스크 상황상 긴급으로 물어봄.

**5) attempt4 전에 필요한 것 — 무작정 재시도 금지**
같은 TF 동결이 재현되면 또 disk만 채우고 무효 결과가 나온다. 재시도 전에:
(a) 감시스크립트의 `timeout -k` 버그부터 고치고, (b) TF 동결 원인을 최소한
한 번은 더 조사(예: exp_drone_tf_bridge 로그를 별도 파일로 분리해 1043s
직전 마지막 정상 메시지 확인, 또는 이 VM의 리소스 한계 때문인지 CPU/메모리
추이와 상관관계 확인)하는 게 안전하다고 판단 — 이것도 사용자 확인 필요
(정지조건 2번: "velocity 비행이 불안정할 때"에 해당한다고 봄. attempt1
climb-rate, attempt2 VM크래시에 이은 **세 번째로 다른 종류의 실패**라
그냥 또 재시도하는 것보다 원인 규명이 우선이라고 판단).

## ✅ GLIM PPA 설치 완료 + 네이티브 실행 조사 (docker → apt 전환 진행 중)

사용자가 직접 터미널에서 PPA 등록 + 패키지 설치를 완료함(`ros-jazzy-glim`,
`ros-jazzy-glim-ros` 1.2.2-0noble, arm64 — `dpkg -l`로 설치 확인).

**실행파일**: `ros2 pkg executables glim_ros` → `glim_rosnode`(라이브 구독,
기존 Docker `run_glim.sh`가 쓰던 것과 동일 모드), `glim_rosbag`(bag 배치 처리,
이번엔 Docker처럼 `ros2 bag play`를 별도로 띄울 필요 없이 이걸로 대체 가능해
보임), 그 외 `map_editor`/`offline_viewer`/`validator_node`.

**설정 경로 확인**: `-p config_path:=<glim_config 절대경로>` 로 우리 리포의
기존 `glim_config/`(CT+passthrough+pose_graph, Docker에서 쓰던 것과 완전히
동일한 파일)를 그대로 재사용 가능 — 별도 config 없이 절대경로만 바꿔주면 됨.
스모크테스트(`glim_rosbag /nonexistent_bag_path --ros-args -p config_path:=...`)
결과: `load libodometry_estimation_ct.so` / `libsub_mapping_passthrough.so` /
`libglobal_mapping_pose_graph.so` 전부 정상 로딩(Docker의 GPU 로딩 실패 문제가
CPU 조합에서는 재현 안 됨) — **CT+passthrough+pose_graph 조합이 네이티브에서도
그대로 작동함을 확인.**

**⚠️ 미해결 — bag 경로 인자 전달 방식**: 존재하지 않는 경로를 줬는데도 에러 없이
"bag_filenames:" 목록이 빈 채로 실행이 이어지다가, 실제 라이브 시각(당시 진행
중이던 attempt3 비행의 sim 시각 1022~1035s대)과 정확히 일치하는 데이터를
받기 시작함 — 즉 **없는 bag 대신 라이브 토픽(`/drone/imu`,`/drone/points`)을
그대로 구독해버린 것으로 보인다.** `glim_rosbag input_rosbag_path`가 usage
문자열엔 있지만 내가 준 위치인자가 실제로 파싱되는지 불확실 — 진짜 존재하는
bag으로, **다른 라이브 비행이 돌고 있지 않을 때** 다시 검증해야 함(라이브
데이터와 섞여 오판하지 않도록). 25초 후 timeout으로 정상 종료시킴 —
attempt3 비행 자체에는 지장 없었음(진행률 로그로 확인).

**✅ bag 경로 인자 미스터리 해결(GitHub 소스 확인, 재실행 없이 정적으로 확인)**:
`glim_rosbag.cpp`(https://github.com/koide3/glim_ros2/blob/master/src/glim_rosbag.cpp)
는 `argv[1..]`을 `glob()`으로 파일 매칭한다 — 내가 준 `/nonexistent_bag_path`는
매칭 0건이었는데도 하드 에러 없이 그대로 진행했고(이게 "bag_filenames:" 빈
목록의 정체), 그 상태에서 노드에 살아있던 라이브 구독이 마침 돌고 있던
attempt3 토픽을 주워온 것 — **실제 존재하는 bag 경로를 주면 glob이 정상
매칭되어 그 파일을 읽는다.** 별도 fallback 로직이 아니라 "매칭 0건이어도
안 죽는다"는 관대한 처리였을 뿐. `auto_quit` 파라미터가 있어 재생이 끝나면
자동 종료되는 것으로 보임(기본값 확인 필요하나 스크립팅엔 유리).
→ **attempt3 완주 후**: `ros2 run glim_ros glim_rosbag bags/velocity_4m_attempt3
--ros-args -p config_path:=$(pwd)/glim_config` 로 바로 실행하면 됨. 추가 검증
불필요, 실제 bag으로 바로 정식 실행.

세션이 끊겨도 이어받을 수 있도록, 단계가 끝날 때마다 여기에 요약을 추가한다.

## 환경/설정
- 작업 위치: `/home/hyunwoo-chae/AG-CoNav-test_main` (test_main 브랜치 전용 git worktree,
  원래 브랜치 `brian_test`의 uncommitted 변경을 건드리지 않기 위해 분리했다).
  원본 리포는 `/home/hyunwoo-chae/AG-CoNav` (brian_test, 손대지 않음).
- 이유: brian_test 체크아웃에는 `src/agconav_test_worlds`가 아예 없고, 기존
  `install/`도 stale(예: `drone_pose_controller.yaml`의 `service_timeout_sec`가
  test_main의 5.0이 아니라 구버전 1.0)이라 실험에 쓸 수 없었다. worktree를 새로
  만들어 test_main HEAD(`44b0190`)를 그대로 체크아웃하고 처음부터 빌드한다.
- ROS2: Jazzy (`/opt/ros/jazzy`), Gazebo Sim 8.11.0, colcon 확인됨.
- 하드웨어: 4 core / 5.8GB RAM — RESULTS.md 실험은 6-core 기준이었고 거기서도
  CPU 포화 문제가 있었다. 이 환경은 더 열악하므로 real-time factor 저하에 더
  주의해야 한다.
- GLIM: 1단계 조사대로 미설치(도커 이미지 없음, GPU 없음, docker 그룹 권한 없음).
  4단계 전에 설치 필요. **[갱신] Docker(koide3/glim_ros2)는 amd64 전용이라 이
  ARM64 VM에서 애초에 불가 — 사용자 지시로 Docker 대신 PPA(apt,
  ros-jazzy-glim-ros, arm64+noCUDA 공식 지원)로 설치 방식 전환. 아래
  "GLIM PPA 전환" 절 참조.**
- 아키텍처: **aarch64(ARM64)** — 크래시 복구 조사 중 처음 확인함(이전엔 미확인).

## 진행 상태
- [x] 2단계: 표면 높이 모델 + 경로 3개 생성 + 안전검증 (경로 3개 파일 자체는 그대로 보존)
- [x] 3단계: teleport 드라이런 — **4m만 유효**(아래 참조)
- [~] 4단계: velocity 실비행 + GLIM — **4m만**. 4-1 attempt2가 VM 크래시로 중단,
  attempt3 재실행 중(아래 "크래시 복구" 절 참조). 4-2 GLIM은 Docker→PPA(apt)
  전환 진행 중 — PPA 저장소 등록에 sudo 필요, 사용자 조치 대기.
- [ ] 5단계: Module A/F 채점 + wheel FN 비교 — **84m 기준선 vs 5m+4m + GLIM 열
  복원(사용자 지시로 GLIM 제외안 취소, 원래 계획대로 복원)**

## 🟡 GLIM PPA 전환 (Docker amd64 블로커 해결책, 사용자 지시)

사용자 확인: koide3 GLIM은 Docker(amd64 전용)와 별개로 PPA(apt) 배포판을
제공하며 Ubuntu 24.04(noble)/22.04에 대해 amd64+arm64를 공식 지원한다
(https://koide3.github.io/glim/installation.html). 이 VM은 Ubuntu 24.04
(ROS2 Jazzy) + GPU 없음이므로 CUDA 없는 조합이 정확히 맞는다.

**진행 상황**:
- `curl`, `gpg` 이미 설치돼 있어 별도 설치 불필요(확인 완료).
- PPA 저장소는 아직 미등록. 등록 스크립트(`https://koide3.github.io/ppa/setup_ppa.sh`)
  내용을 먼저 읽어 확인 — 전체가 `EUID -ne 0`이면 즉시 종료하도록 짜여 있어
  **반드시 root 권한 필요**(`/etc/apt/trusted.gpg.d/`, `/etc/apt/sources.list.d/`
  쓰기 + `apt update`). 이 세션은 `sudo -n true`도 비밀번호를 요구해 실패
  (기존 docker 그룹 블로커와 동일한 성격의 제약) — **직접 실행 못 함.**
- ⏸ **사용자 조치 필요**: 터미널에서 `!` 접두사로 다음을 실행해달라:
  ```
  curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
  ```
  등록되면 `apt-cache policy ros-jazzy-glim-ros` 자체는 sudo 없이 내가 바로
  실행해 arm64+noCUDA 조합이 실제로 존재하는지 확인하고(사용자 지시 1단계),
  나오면 곧장 `sudo apt install ...` 패키지 설치로 진행한다(사용자가 이미
  이 경로를 승인함) — 이 설치도 sudo가 필요해 마찬가지로 사용자가 위 명령과
  함께, 또는 그 다음에 별도로 실행해줘야 할 가능성이 높다(설치 커맨드는
  이 조사 완료 후 정확한 패키지 목록으로 다시 안내하겠음).

## 🔴 크래시 복구 — 어디까지 유효하고 어디부터 재실행하는지 (VM 재부팅 후 조사)

VM이 4단계-1 attempt2 비행 도중 크래시 후 재부팅됐다. 재개 전, 디스크에 남은
산출물의 수정 시각을 rate-limiter 4m 경로 재생성 시각(아래 "정지조건 3번 해결"
절, 파일 mtime 기준 15:36)과 대조해 유효성을 판정했다.

**1) 2단계 경로 파일 — 전부 유효, 그대로 재사용**
```
path_100x100_5m_2m.yaml   13:39  (원본, rate-limiter 미적용 — 애초에 2m은 실험 제외 확정이라 무관)
path_100x100_5m_3m.yaml   13:39  (원본, 3m도 실험 제외 확정이라 무관)
path_100x100_5m_4m.yaml   15:36  (rate-limiter 재생성판 — "정지조건 3번 해결" 절과 시각 일치)
```
2m/3m은 애초에 4~5단계에서 안 쓰므로 재생성 대상도 아니었고, 4m은 재생성 시각이
기록과 정확히 일치. **재실행 불필요.**

**2) 3단계 드라이런(4m, rate-limiter 적용판) — 유효, 그대로 재사용**
```
run_results/logs/dryrun_teleport_4m_v3.log   16:27  (재생성 15:36 이후)
run_results/dryrun_4m.md                     16:40  (재생성 15:36 이후, PROGRESS 마지막 기록 16:41 직전 완료)
```
둘 다 4m 경로 재생성(15:36) *이후* 시각이라 rate-limiter 적용판에 대한 유효한
결과다. 커버리지 99.6%, 완주 정상 — **재실행 불필요.**

**3) 4단계-1 velocity 비행 attempt2 — 무효, attempt3로 재실행**
`run_results/logs/velocity_4m_attempt2.log`(mtime 17:02, PROGRESS.md 마지막 기록
16:41보다 늦음 = 이 세션이 기록하지 못한 진행분)를 직접 열어 확인:
- 마지막 로그 줄이 `waypoint 2720 / 5251 도달`(진행률 51.8%)에서 **정상 종료
  메시지 없이 뚝 끊김**. 완주 메시지도, 에러/트레이스백도, SIGINT 처리 로그도 없다
  — 프로세스가 그 순간 통째로 죽은 크래시 시그니처.
- 로그 안 타임스탬프(ROS 시각 `1786780963` → 2026-08-15 17:02:43 KST)가 로그
  파일 mtime(17:02)과 정확히 일치 — 그 순간 VM이 죽은 것으로 확정.
- bag(`bags/velocity_4m_attempt2/`, 6개 mcap 파일 총 12GB)도 마지막 파일이
  17:02에 끊겨 있고, 그다음 분할파일(`_6.mcap`)은 0바이트로 생성만 되고 못 씀 —
  같은 크래시 순간의 흔적.
- **비행이 완주되지 않았으므로 GLIM 입력으로도, 5단계 채점 입력으로도 쓸 수
  없다.** 재사용 불가 판정 → 삭제(디스크 확보) 후 **attempt3**로 처음부터
  재실행한다. 런치 인자는 attempt2와 동일(`flight:=velocity
  path_file:=path_100x100_5m_4m.yaml cruise_speed_mps:=4.0
  bag_output:=.../bags/velocity_4m_attempt3`), 타임아웃도 기존 확정값(4500s=75분)
  그대로 유지 — 새로운 설계 판단 아님, 단순 재시도.
- 재부팅 후 잔여 프로세스 없음 확인(`pgrep -af "ros2 launch|gz sim|gz-sim|
  parameter_bridge|ros2 bag record|drone_"` 결과 없음) — 정리 절차 별도로
  필요하지 않았다.

**4) 4단계-2 GLIM — 아직 미착수. docker 권한 블로커는 여전히 미해결**
`id -nG`에 `docker` 그룹 없음, `docker info` 여전히 `permission denied`(server 쪽) —
재부팅됐지만 사용자가 `sudo usermod -aG docker $USER`를 실행하지 않은 것으로
보인다. (아래 5번의 더 근본적인 블로커 때문에 이 문제는 지금 당장은 부차적임.)

**5) 🔴 신규 치명적 발견 — 이 VM은 ARM64인데 GLIM 이미지는 전부 amd64 전용**
사용자가 크래시 전부터 물어봤던 질문에 대한 답:
- `uname -m` → **aarch64**. (이전 조사에선 아키텍처를 확인한 적이 없었다.)
- `docker manifest inspect --verbose koide3/glim_ros2:jazzy_cuda13.1` →
  `"platform": {"architecture": "amd64", "os": "linux"}` — **manifest list가
  아니라 단일 플랫폼(amd64) 이미지**, arm64 변형이 없다.
- Docker Hub API로 리포지토리 전체 태그(`jazzy_cuda13.1`, `jazzy_cuda12.5`,
  `jazzy`, `humble`, `humble_cuda12.2`) 확인 — **5개 태그 전부 amd64뿐, arm64
  태그 자체가 리포지토리에 없음.**
- 이 VM에 `qemu-x86_64` binfmt 에뮬레이션도 등록돼 있지 않음
  (`/proc/sys/fs/binfmt_misc/qemu-x86_64` 없음) — 설령 등록한다 해도 CUDA
  이미지를 GPU 없는 ARM64 VM에서 에뮬레이션으로 돌리는 건 SLAM 워크로드
  특성상 사실상 비현실적(속도·안정성 모두)이라고 판단.
- **결론: docker 그룹 권한 문제를 사용자가 지금 당장 해결해줘도, 이 이미지
  자체가 이 VM에서 pull조차 안 된다.** 이건 확정 범위("GLIM CT+passthrough+
  pose_graph 조합")를 이 하드웨어에서 물리적으로 실행할 방법이 없다는 뜻이라
  **정지조건 3번**에 해당한다고 판단해 여기서 멈추고 보고한다. 가능한 선택지
  (사용자 결정 필요, 그중 어느 것도 확정 범위 안이 아니라 임의로 고르지 않음):
  (a) GLIM을 ARM64용으로 소스 빌드, (b) x86_64 VM/머신으로 이 4-2 단계만
  옮겨서 실행, (c) 4-2(GLIM)를 이번 실험 범위에서 제외하고 5단계를 GLIM 없는
  형태로 축소, (d) 다른 SLAM 대안으로 교체. **사용자 확인 전까지 4-2는 진행하지
  않는다.**

**6) 디스크 — 이번 크래시가 디스크 고갈 때문이었다는 증거는 없음, 그래도 안전장치 추가**
재부팅 후 현재 `df -h ~`: 78G 중 43G 사용(59%), 31G 여유 — 위험 수준 아님.
attempt2가 크래시 전까지 만든 bag은 약 21분간 12GB(≈9.5MB/s, RESULTS.md의
"200초에 54GB"(≈270MB/s) 폭주 사례와는 자릿수가 다른 정상적인 증가 패턴) —
이번 크래시의 원인이 디스크였다는 근거는 못 찾았다(VM 자체 크래시로 보임,
원인 불명). 그래도 사용자 지시대로 **attempt3부터는 상태로그 갱신마다
`df -h ~`를 함께 기록하고, 사용량이 80%를 넘는 순간 즉시 비행을 중단(SIGINT→
TERM 에스컬레이션 후 잔여 gz sim 프로세스까지 정리)하고 실행을 멈춘 뒤
보고하도록** `run_results/logs/status_log_attempt3.log`에 60초 주기로 기록하는
감시 스크립트를 붙여 실행한다.

**정리(디스크 확보) — 삭제는 사용자 확인 대기 중, 아직 실행 안 함**: 재사용
불가로 판정된 `bags/velocity_4m_attempt2/`(12GB)와, 이미 분석이 끝나 산출물
(`dryrun_4m_v2_rootcause.md` 등)로만 남기면 되는 `bags/probe2_4m/`(3.9GB, 3단계
진단용, 분석 완료됨)를 삭제해 여유 공간을 넓히려 했으나, `rm -rf`가 자동 승인
범위 밖이라 차단됨 — 사용자에게 별도로 확인받아야 한다. 다만 삭제 없이도 현재
여유 공간(31G)만으로 attempt3 한 사이클(예상 ~24GB) 진행에는 지장이 없다고
판단해, 삭제 확인을 기다리는 동안 **attempt3를 먼저 시작**했다
(`run_results/run_attempt3_monitored.sh`, 백그라운드 실행 중, 60초 주기로
`run_results/logs/status_log_attempt3.log`에 진행률+`df -h ~` 기록, 80% 도달 시
자동 중단).

## ✅ 정지조건 3번 해결 — rate-limiter로 4m 경로 재생성

사용자 승인(옵션1: rate-limiter로 z-프로파일 스무딩 후 재생성)에 따라
`run_results/generate_paths.py`에 `rate_limit_z()`(slope-limited majorant, 2-pass
max-plus 구성)를 추가하고 4m 경로만 재생성했다:
- 설계 상한: `climb_speed_mps` 예산(6.0, velocity_path_follower.py 기본값)의
  **70%인 4.2 m/s**로 제한(안전계수 0.7 — 실제 컨트롤러 추종 지연/오버슈트 여유).
  4m/s 순항 기준 `MAX_SLOPE = 4.2/4.0 = 1.05`.
- 결과(재생성 로그): `max_req_vz=4.20 m/s`(설계상한과 정확히 일치, 정상), **예산초과
  세그먼트 0개**(attempt1 스캔 때 170개였던 것과 대비), 최소 클리어런스도
  4.650m로 오히려 개선(기존 3.033m) — 건물 주변에서 미리 올라가는 만큼 여유가
  더 생긴 것.
- 최종 웨이포인트 5,251개(기존 5,260개와 큰 차이 없음), 총 경로 길이 2,880.0m,
  예상 순수비행 720.0s(12분, 기존 대비 오히려 짧아짐 — 이전엔 자동보정으로 삽입된
  9개 웨이포인트가 있었으나 이번엔 0개).
- **2m/3m은 건드리지 않음**(이미 실험 제외 확정, yaml 파일 무수정) — 다만 이번
  4m 재생성이 `run_results/path_5m_safety_report.md`를 스크립트가 통째로
  다시 쓰면서 2m/3m 절을 날렸길래, rate-limiter *이전* 로직으로 재계산해 원래
  수치 그대로(2m: final=10322 corr=21 len=6041.2m / 3m: final=6878 corr=11
  len=4056.6m, 전부 이전과 동일) 복원해 다시 끼워넣었다(`rebuild_2m_3m_report_section.py`,
  yaml 파일은 전혀 건드리지 않고 텍스트 섹션만 재계산). 이 2m/3m 절이 rate-limiter
  적용 이전 버전이라는 점을 보고서 안에 명시해뒀다.
- `path_5m_visualization.png`도 갱신(4m 궤적만 바뀜, 2m/3m은 원본 yaml 그대로라 동일).
- **3단계 teleport 드라이런 재확인 완료**(`dryrun_teleport_4m_v3`, 1097s):
  경로 완주 정상, 박스 기준 커버리지 **99.6%**(rate-limiter 적용 전과 동일),
  이상치 0%, 고도 범위 0.746~8.918m 정상. `run_results/dryrun_4m.md` 갱신함
  (이 파일이 최종본 — attempt1 정지 전 버전은 git 히스토리에만 남음).
  → **4단계 재시도(attempt2)로 진행.**

## ⏸ (해결됨, 기록 보존) 4단계-1 attempt1에서 정지조건 3번(설계 갈림길) 발동

**attempt1 결과: 570초/5,260개 중 1,318번째 웨이포인트에서 고도가 설계값(8.4m)에서
30m 이상(38.5m) 벗어나 팔로워가 자체 안전중단.** 로그: 웨이포인트 1305까지는
선이탈 0.1m 이하로 완전히 정상 비행하다가, 1309부터 선이탈이 1.6→2.9→4.3→...→12.0m로
**단조 발산**(진동이나 일시적 흔들림이 아니라 회복 없이 계속 벌어짐).

**Root cause 확인(재시도 전에 먼저 진단함, 근거 있는 판단):** 웨이포인트 1303→1304
구간에서 z가 7.40m→11.37m로, **수평 0.5m 이동 중 수직 3.97m 상승**을 요구한다
(건물 가장자리 — 지형 위 AGL에서 건물 지붕 위 AGL로 전환). 4m/s 순항 기준 이 구간
통과 시간은 0.125초이므로 **요구 수직속도 ≈ 31.8 m/s** — `velocity_path_follower.py`
자체의 `climb_speed_mps` 예산(6.0)의 5배가 넘는다. 이건 자세제어 게인 문제가 아니라
**경로 자체가 이 드론이 물리적으로 따라갈 수 없는 상승률을 요구**하는 것이다(어떤
attitudeGain/angularRateGain 조합도 30 m/s 수직 속도를 6.0 m/s 예산의 컨트롤러로
따라잡게 할 수 없다).

**전체 경로를 스캔해 확인 — 일회성이 아니라 구조적 결함:** 4m 간격 경로 전체
5,259개 세그먼트 중 **170개(3.2%)가 climb_speed_mps(6.0) 예산을 초과**하며,
최악의 경우 **93.4 m/s**를 요구한다. 건물 20채 전부의 가장자리에서 반복적으로
발생하는 패턴 — 방금 멈춘 지점(#1303~1304)을 어떻게든 넘긴다 해도 남은 169개
지점 중 어딘가에서 다시 같은 방식으로 발산할 것이 거의 확실하다.

**왜 3단계(teleport 드라이런)에서는 안 걸렸나:** SetEntityPose 순간이동은 물리
가속도·속도 제한이 없어(공간 기하만 확인) 이 결함이 드러나지 않았다. 2단계
안전검증(waypoint 사이 0.2m 샘플링)도 "위치가 지표면+3m 이상인가"만 봤지
"그 위치에 그 타이밍으로 도달 가능한가"(운동학적 실현 가능성)는 애초에 검증
항목이 아니었다 — 이건 2단계 안전검증의 범위 밖 결함이라 정지조건 1번(안전검증
실패)이 아니라 3번(설계 갈림길)에 해당한다고 판단했다.

**왜 "3~4회 자동 재시도"(정지조건 2번 절차)를 그대로 따르지 않았나:** 이미 확정적
증거(전체 경로 스캔)로 게인 튜닝이 원인이 아님을 확인한 상태에서 재시도를 반복하는
건 결과가 뻔한 시도에 매번 ~10분 이상(시뮬 재시작+비행)을 태우는 것이라 판단해,
진단이 끝난 시점에 바로 멈추고 보고하는 쪽을 택했다.

**제안하는 수정 방향(사용자 확인 필요 — 이게 왜 정지조건 3번인지):** 스트립 방향
0.5m 간격으로 "그 지점의 지표면+5m"를 그대로 웨이포인트 z로 쓰는 2단계 로직 자체를
바꿔야 한다. 예: 건물 경계 근처에서 z 프로파일에 **상승률 상한(climb_speed_mps
기준)을 두는 rate-limiter**를 적용해, 건물에 도달하기 전부터 미리 상승을
시작하고 지난 뒤에 서서히 하강하게 만드는 방법(항상 필요 최소고도 이상만
유지하도록 클리핑하므로 안전마진은 절대 깨지지 않고, 건물 주변에서 잠깐
더 높이 나는 것으로 완화됨). 이건 "5m AGL 방법B"를 매 지점에서 문자 그대로
지키는 것을 살짝 완화하는 결정이라 사용자 확인 없이 진행하지 않는다.

## 4단계-1 진행 상황 (velocity 실비행, 4m 간격)

- world: `Seongdong_gu_100x100_dynamic`(RESULTS.md 10절 튜닝값 적용된 사본, flight:=velocity가
  자동 선택), cruise_speed_mps:=4.0(launch에 새로 추가한 인자로 명시 지정 — 안 하면 launch
  기본값 8.0이 나가서 확정 스펙 위반이었을 것, 발견해서 수정함).
- **attempt1 첫 시도부터 안정적** — RESULTS.md 10절 튜닝값(스폰위치/forceConstant/
  attitudeGain/angularRateGain)이 별도 조정 없이 그대로 통했다. 125초 시점: waypoint
  256/5260, 선 이탈 0.19~0.36m, 위치 (36.66, -161.89, 8.88) — 추락/충돌/발산 징후 없음.
  정지조건 2번(불안정) 해당 없음.
- **속도가 예상보다 훨씬 느림(타이밍 문제, 안정성 문제 아님)**: 순항 4m/s를 지정했지만
  0.5m 간격으로 촘촘히 찍은 웨이포인트가 `velocity_path_follower.py`의
  `arrive_radius_m`(기본 1.5m) 감속 로직과 부딪혀 사실상 계속 "도착 임박" 감속 상태에
  머무는 것으로 보임(관측: 125초에 256개 웨이포인트 = 실질 평균 ~1.3 m/s). 전체
  5,260 웨이포인트 완주 예상 시간 ≈ 2,568s(43분) — 원래 지시문의 stage-3 20분 기준과도
  다른 이슈이고, 이건 4단계 완주 자체를 막지는 않으므로(안정적으로 계속 진행 중)
  타임아웃만 넉넉히(4,500s=75분) 늘려서 그대로 완주를 기다린다. 코드/파라미터를
  수정하지 않음(확정 스펙 밖 변경을 피하기 위해 — 필요하면 나중에 팀 논의 항목으로
  남길 사항이지 지금 내가 바꿀 부분은 아니라고 판단).

## ⚠️ 4단계-2 블로커 — GLIM Docker 이미지 pull 불가 (사용자 조치 필요)

1단계에서 이미 확인한 블로커가 그대로 유효함: `docker info` permission denied,
`id -nG`에 docker 그룹 없음, `sg docker -c`/`sudo -n`모두 비밀번호 요구 — 이 세션
권한으로는 `docker pull koide3/glim_ros2:jazzy_cuda13.1`을 실행할 방법이 없다.
4-1 비행이 끝나는 대로 bag은 확보되지만, GLIM을 실제로 돌리려면 사용자가 터미널에서
직접(`!` 접두사로) 다음 중 하나를 해줘야 한다:
```
sudo usermod -aG docker $USER   # 이후 새 로그인 셸에서 적용됨
# 또는
sudo docker pull koide3/glim_ros2:jazzy_cuda13.1   # sudo로 직접 pull
```
GPU 없는 환경이 확인됐으므로 실행 시 `--cpu` 옵션이 필요하다(`run_glim.sh`가 이미
처리). 4-1이 끝나는 시점까지 이 블로커가 안 풀리면 4-2에서 실제로 멈추게 된다.

## ⚠️ 최종 범위 — 사용자 지시로 2m·3m 모두 제외, 4m 간격 하나만 진행

시간 문제로 사용자가 두 차례에 걸쳐 범위를 축소했다. 최종적으로 **이번 실험은
"5m AGL + 스트립간격 4m" 조합 하나만 4~5단계까지 끝까지 진행한다.**

- **2m**: 2단계 안전검증까지만 실행하고 실험 시작 전 사용자가 제외 결정.
  `path_100x100_5m_2m.yaml`, 안전보고서 2m 절, 시각화의 2m 궤적은 유효한
  산출물이라 그대로 보존(삭제 안 함). 3단계 이후 어디에도 쓰지 않음.
- **3m**: 3단계 드라이런을 실행하던 도중 사용자가 직접 중단 지시. **이건 실패나
  데이터 손상이 아니라 의도적 중단**이다 — `path_status`가 True가 되기 전에
  launch를 SIGINT로 정상 종료했고(gz sim 잔여 프로세스도 별도 SIGINT로 마저
  정리, 잔여 프로세스 없음 확인 완료), 미완성 bag(7.6GB, path_status 도달 전
  상태)은 디스크 확보를 위해 삭제했다. **3m 관련 산출물은 하나도 없고, 이후
  어떤 단계에서도 3m을 언급/사용하지 않는다.** `path_100x100_5m_3m.yaml`
  자체(2단계 산출물)는 유효하므로 보존.
- **4m**: 3단계 드라이런은 이미 완료된 기존 결과(`run_results/dryrun_4m.md`,
  root-cause 조사 포함, 박스 기준 커버리지 99.6%)를 그대로 재사용. 재실행하지
  않음. 4~5단계는 **4m만** 진행.

**5단계 최종 비교표는 2행만**: "84m 실제비행+GLIM(RESULTS.md 기준선)"과
"5m AGL + 4m 간격" — 그 외 행은 만들지 않는다.

**중단 절차(참고용 기록)**: launch 프로세스(ros2 launch)에 SIGINT → 죽지 않으면
TERM 순으로 에스컬레이션 → cascade shutdown 후에도 `gz sim` 바이너리가 별도
프로세스로 남는 경우가 있어(이번에도 재현됨) 그것도 별도로 SIGINT/TERM →
`pgrep -af "ros2 launch|gz sim|gz-sim|parameter_bridge|ros2 bag record|drone_*"`
로 잔여 프로세스 0개 확인. 4단계에서도 실행/재시도마다 이 절차를 그대로 반복한다.

## ✅ 해결됨 — "낮은 커버리지" root-cause 조사 결과 및 최종 결정

사용자 지시로 root-cause를 먼저 파본 결과, **정지조건 3번을 취소하고 계속 진행하기로
확정**. 조사 과정과 결론:

**1) 센서 사거리(170m) 초과 가설 — 기각(직접 실측으로 확인)**
`run_results/probe_transformed.py`로 drone_elevation_mapper와 동일한 파이프라인
(isfinite 필터 → min_range 2.5m 자기반사 필터 → TF 변환)을 재현해 100개 스캔·
144만 점을 직접 검사: **170m 초과 점 0개** (관측된 최대 range는 79.8m). gz의
gpu_lidar가 사거리 밖은 Inf로 채우고(코드 주석에 이미 명시, isfinite로 걸러짐)
그 경로는 완전히 정상 작동 중이었다.

**2) 실제 원인 — 박스 밖 유효 셀의 공간 분포로 확인**
`run_results/analyze_bag.py`를 확장해 GridMap의 각 유효 셀이 박스 경계로부터
얼마나 떨어져 있는지 히스토그램을 뽑음(4m 간격 드라이런 재실행, `dryrun_4m.md`):
- 박스 밖 유효 셀 32,509개 중 **87.7%는 경계에서 5m 이내** — 5m AGL·4m 간격에서
  스와스(swath) 폭이 좁아 인접 스트립 간 그레이징 각도 관측이 박스 가장자리를
  살짝 넘는, 정상적이고 무해한 현상.
- 나머지 꼬리(2.7%, 20m~98.1m)는 **TF/자세 보간 불안정으로 설명됨**: 우리 lawnmower
  경로는 스트립마다 180° yaw 반전 코너가 있다(4m 간격=25회, 84m 기준선 경로는
  전체 10 waypoint 중 4회뿐). 코너에서 발행되는 pose가 한 틱(0.1s) 안에 yaw
  ~0°→~180°로 급변하는데, 그 찰나에 스캔 타임스탬프가 걸리면 tf2 Buffer의
  quaternion 보간이 두 정반대 회전 사이에서 불안정해져(수학적으로 정의역 경계)
  그 스캔의 점들이 로컬 거리·고도값은 정상인데 맵 좌표계에서 엉뚱한 방향으로
  튄다(최악 사례: (79.20, 32.00), 고도 4.744m로 값 자체는 멀쩡하나 박스에서
  98m 이탈). 84m 기준선은 코너가 4회뿐이라 이 현상이 있어도 거의 안 보였을
  뿐, 우리가 새로 만든 결함이 아니라 **경로의 턴 횟수에 비례해 커지는, 기존
  파이프라인에 잠재해 있던 현상**으로 판단.
- 두 원인 모두 "센서 사거리 안의 정상 관측"이라 사용자가 제시한 분기 중
  **2번(Module A는 건드리지 않고 채점 스크립트만 박스 기준으로 수정)**에 해당.

**3) "heightmap skirt" 가설 — 사실상 기각**
박스 밖 셀의 87.7%가 5m 이내로 몰려 있고 나머지는 코너-보간 좌표 이상으로
설명되므로, 지형 경계의 인공 스커트 지오메트리가 별도로 반사를 만들어내는
정황은 찾지 못했다. (100% 확증은 아니지만 관측된 패턴을 스커트 가설로 설명할
필요가 없어졌다.)

**4) 조치 — Module A(`drone_elevation_mapper.py`)는 무수정, 채점만 박스 기준으로**
`run_results/analyze_bag.py`에 "박스 기준 커버리지" 절을 추가:
분모를 원본 GridMap 전체 셀이 아니라 **100x100 박스의 예상 셀 수(1,000,000)**로
고정하고, 분자도 그 박스 안에 있는 유효 셀만 센다. 이 필터는 코드 주석과
여기 모두에 **"이번 100x100 실험 전용, 500x500 확장 시 재사용 금지"**라고
명시해뒀다(하드코딩된 BOX 상수를 그대로 재사용하면 안 됨).

**5) 재채점 결과 — 애초에 문제가 아니었다**
4m 간격 드라이런을 박스 기준으로 다시 채점: **커버리지 99.6%** (995,551/1,000,000
셀). 원본(그리드 팽창 포함) 지표였던 21.4~25.0%는 순전히 분모 오염 때문이었고,
실제 100x100 박스 안 관측은 84m 기준선(59.51%)보다 오히려 훨씬 낫다 — 5m AGL +
4m 간격의 좁은 스와스가 촘촘히 겹치며 전 구역을 거의 빠짐없이 스캔한 결과로
해석된다. 박스 안 유효 셀의 고도값도 0.411~8.859m로 지표면 모델 기대범위
(1.202~8.780m)와 사실상 일치, 이상치 0%.

**결론: 정지조건 3번 해제, 3단계 계속 진행.** 3m/2m 간격도 같은 방식(박스 기준
채점)으로 드라이런하고, 4~5단계도 이 채점 로직을 그대로 쓴다.

## ⏸ 정지 — 3단계에서 정지조건 3번(경로 자체 결함/설계 갈림길) 발동

**4m 간격 드라이런(teleport, Seongdong_gu_100x100) 결과: Module A 커버리지 25.0%**
— RESULTS.md 6절의 84m 기준선(59.51%)보다 뚜렷이 낮음. 자세한 진단은
`run_results/dryrun_4m.md` 참조. 요약:
- 원인은 새로 생긴 버그가 아니라 **RESULTS.md 9절에 이미 기록된 기존 결함**
  ("모듈 A `_grow_to_fit`에 상한이 없음")과 같은 계열이지만, 5m AGL에서 그 결함의
  악영향(그리드가 의도한 100x100보다 훨씬 크게 부풀어 커버리지%를 희석시키는 정도)이
  84m 대비 훨씬 크다(그리드 y방향이 의도한 100m의 2.6배 vs 84m 기준선은 1.12배).
- 원인 후보(미확정): 5m AGL은 지면에 훨씬 가까워 얕은 각도로 나가는 LiDAR 빔이 박스
  경계 밖까지 스치듯 도달해 반환되는 경우가 84m보다 많을 가능성.
- (1차 분석에서 "z가 -45~+38m까지 튄다"고 오판했던 것은 내 분석 스크립트가 `/drone/points`
  의 센서 로컬 프레임 좌표를 world 좌표로 잘못 해석한 방법론 오류였음 — 정정 완료.
  실제 GridMap에 쌓인 고도값 자체는 정상 범위였다.)

**3단계 지시문이 명시적으로 예시로 든 정지 트리거("극단적으로 낮은 커버리지")에
해당한다고 판단해 여기서 멈춘다.** 3m/2m 드라이런, 4~5단계는 사용자 지시를
기다린 뒤 진행.

**산출물(지금까지)**:
- `run_results/dryrun_4m.md` — 4m 간격 드라이런 상세 분석.
- `run_results/analyze_bag.py`, `run_results/scan_point_extremes.py`,
  `run_results/probe_points_once.py` — bag/포인트클라우드 진단 도구(재사용 가능).
- `src/agconav_test_worlds/launch/experiment.launch.py` — `path_file` 런치 인자 추가
  (기본값 `path_100x100.yaml`로 기존 동작 100% 보존, 5m AGL 경로 파일 선택 가능하게 함).
  이 변경은 정지조건 3번과 무관한 순수 배관(plumbing) 변경.
- 4m 드라이런/probe용 bag은 분석 후 디스크 확보를 위해 삭제함(재현 가능한 산출물이라
  원본 보존 불필요하다고 판단 — bag_output/maps_output 인자로 언제든 재생성 가능).

## ⚠️ 알려진 블로커 — 4단계 GLIM 설치 전에 사용자 조치 필요
`docker info`가 permission denied, `sg docker -c ...`도 그룹 비밀번호를 요구해서
실패(`sudo -n true`도 비밀번호 필요 — 이 세션엔 비밀번호를 모름). 즉 **현재 세션
권한으로는 docker 그룹에 진입할 방법이 없어 GLIM 이미지를 pull할 수 없다.**
4단계 진입 전에 사용자가 터미널에서 직접 다음을 실행해야 한다(비밀번호 필요라
내가 대신 못 함):
```
sudo usermod -aG docker $USER
# 그 후 이 세션이 아니라 새 로그인 셸에서 docker 그룹이 적용됨.
```
지금은 3단계(GLIM 불필요)를 계속 진행하고, 4단계 진입 시점에 이 블로커를
사용자에게 알린다.

## 2단계 완료 (요약)

**산출물**
- `run_results/surface_model.py` — 지표면 높이 함수(지형 vs 건물 중 최댓값), 0.5m 격자 캐시
  (`run_results/surface_cache.npz`).
- `run_results/generate_paths.py` — lawnmower 웨이포인트 생성 + 안전검증/자동보정.
- `src/agconav_test_worlds/config/path_100x100_5m_{2m,3m,4m}.yaml` — 최종 경로 3개
  (기존 `path_100x100.yaml`은 그대로 보존).
- `run_results/path_5m_safety_report.md`, `run_results/path_5m_visualization.png`.

**지표면 모델 설계 판단**
- 지형: height_map.png를 world의 `<heightmap><size>=100 100 5.249597>`,
  `<pos>=11.4 -116.1 1.202132>`로 쌍선형 디코딩(1단계에서 검증한 gz 공식 그대로,
  `elevation = pixel/img.max()*size.z + pos.z`).
- 건물: buildings.dae를 정점 welding + union-find로 20개 연결요소(=개별 건물)로 분리
  (1단계에서 확인한 `generate_test_world.py`의 `crop_dae()` 로직 재사용). 각 건물의 (x,y)는
  **convex hull**로 포함판정, roof Z는 그 건물 정점 중 최댓값. Hull은 실제 풋프린트의
  상위집합이라 "과소평가 금지" 요구사항과 방향이 일치(넓게 잡을지언정 좁게 잡지 않음).
- 임의 좌표 조회(`SurfaceModel.height_at`)는 그 점을 감싸는 2x2 격자 셀 중 **최댓값**을
  반환(쌍선형 보간 아님) — 건물 가장자리에서 인접 지형 셀과 평균 내어 낮잡는 일을 방지하기
  위한 보수적 설계. 부작용: 격자 셀(0.5m) 내부에서는 지표면이 계단식(구간별 상수)이 됨.
- 실측: 건물 20채, 지표면 고도 1.202~8.780m (지형만은 1.202~6.452m). Convex hull 기준
  건물이 지형보다 높게 잡히는 셀 비율 = 37.8%(0.5m 격자) — 1단계 조사 때 낸 bbox 합산
  근사치(67%, 겹침/직사각형 여유 포함이라 과대추정)보다 낮고 더 정확한 값. 시각화
  (`path_5m_visualization.png`)로 건물 20채 윤곽과 3개 경로가 지표면 위에 정상적으로
  겹치는 것을 육안 확인했다.

**웨이포인트 생성**
- 스트립은 x방향, y방향으로 간격만큼 이동하는 고전 lawnmower. 스트립 내부는 0.5m
  간격(사용자 지시 범위의 촘촘한 쪽 끝)으로 균일 적용 — 건물이 전체 면적의
  1/3 이상을 차지해 "건물 근처만 촘촘히"가 의미 없다는 사용자 지침 반영.
- z = surface_model(x,y) + 5.0m.
- 방향(orientation)은 진행방향 yaw(순방향 스트립 0°, 역방향 스트립 180°)로, 기존
  `path_100x100.yaml`의 컨벤션과 동일하게 구성.

**안전검증 — 실제로 위반을 잡아내고 고친 사례**
- 1차 구현에서 세그먼트 내부 샘플 개수를 `int(seg_len/0.2)`로 계산해 0.5m 세그먼트가
  실제로는 0.25m 간격(3점)으로만 검사됐다. 그 상태에서는 corr=0으로 "전부 안전"하게
  나왔는데, 이게 진짜 안전해서가 아니라 **덜 촘촘히 검사해서 위반을 놓쳤을 가능성**이
  있다고 판단해 `math.ceil()`로 바꿔 확실히 ≤0.2m 간격을 보장하도록 수정 후 재실행했다.
- 수정 후: **2m 간격 경로에서 21곳, 3m 15개, 4m 9곳**에서 실제로 3.0m 마진 위반이
  잡혔고, 위반 지점에 그 지점 실측 지표면+5.0m 웨이포인트를 삽입하는 방식으로
  전부 자동보정됨(재분할 반복 중 재위반 없음 — 정지조건 1 미해당).
- 최종 최소 클리어런스: 2m=3.006m, 3m=3.030m, 4m=3.033m — 전부 3.0m 마진 이상 충족.

**경로별 요약(등속 4 m/s 기준, 가감속 미포함)**

| 간격 | 웨이포인트 | 경로길이 | 예상 순수비행시간 | 턴 | 자동보정 | 최소클리어런스 |
|---|---|---|---|---|---|---|
| 2m | 10,322 | 6,041.2 m | 1510.3 s (25.2분) | 50 | 21 | 3.006 m |
| 3m | 6,878 | 4,056.6 m | 1014.2 s (16.9분) | 33 | 11 | 3.030 m |
| 4m | 5,260 | 3,119.7 m | 779.9 s (13.0분) | 25 | 9 | 3.033 m |

**3단계로 넘기는 주의사항**: 2m/3m 간격 경로는 등속만으로도 20분(1200s)을 넘긴다
(2m=25.2분, 3m=16.9분, 실제로는 코너 감속 등으로 더 길어짐). 사용자가 3단계 드라이런
타임아웃으로 "20분"을 제시했지만 이는 84m 고도의 짧은 왕복 경로 기준이었을 가능성이 커서,
이번 100x100 전체 커버 경로 특성상 비현실적이다. 확정 범위(스트립간격 2/3/4m, 4m/s,
100x100 전체 커버)를 그대로 지키면서 "타임아웃 값" 자체는 실행 파라미터일 뿐이므로,
경로별 예상 비행시간 + 50% 여유를 둔 타임아웃(예: 2m→40분, 3m→27분, 4m→20분)으로
조정해 사용한다. 이 판단은 정지조건 3("확정 범위 밖의 새 선택지")에 해당하지 않는다고
보고 계속 진행한다 — 판단 근거를 여기 남긴다.


## 정지 조건 (반드시 사용자에게 보고 후 대기)
1. 2단계 안전검증 자동보정 반복 후에도 위반이 남는 경로가 있을 때
2. 4단계 velocity 비행이 자동 튜닝 3~4회 후에도 불안정/충돌할 때
3. 확정 범위(5m AGL 방법B, 4m/s, 스트립간격 2/3/4m, GLIM CT+passthrough+pose_graph,
   wheel FN 비교 방식) 밖의 새 선택지가 필요할 때

---

## 📝 디스크 안전 정책 추가 (사용자 저장공간 우려 확인 후, 재비행 도중 보강)

사용자가 재비행 저장공간을 우려해 실측을 요청. 확인 결과:
- bag 증가 속도 실측 ~10MB/s (velocity_4m_5mps_attempt1, 초반 60초 구간 샘플).
- 예상 최종 bag 크기 ~7~9GB (경로 예상 비행시간 700~900s 기준).
- 확인 시점 디스크: 78G 중 37G 사용(50%), 여유 37G — 1회 완주는 충분하나,
  재시도 허용치(3~4회)를 다 쓰면서 실패 attempt bag을 안 지우면
  4×9GB≈36GB로 여유공간을 거의 소진할 수 있음.

**정책**: attempt가 실패해 재시도(attempt2, attempt3...)로 넘어갈 때, 실패한
이전 attempt의 bag 디렉토리를 다음 attempt 시작 전에 삭제해 디스크를 확보한다
(성공한 최종 bag만 남긴다). 디스크 감시 임계값은 기존 80% 대신 70%부터
선제적으로 정리 판단을 시작한다.

이 보강 지시 시점(velocity_4m_5mps_attempt1 진행 중, gz sim + ros2 bag record
PID 27074/27079/27286 확인)까지는 attempt1 하나만 존재, 실패/정리 이력 없음.

## [이번 세션] 0단계 재비행 + 1~2단계 Module A 이식 진행 중

**0단계 판단**: bags/ 가 완전히 비어 있음(이전 세션이 재현 가능하다는 이유로
전부 삭제) — 홈 디렉토리 전체를 find로 검색해도 5m AGL(4m 간격, 5m/s,
velocity) bag이 어디에도 남아있지 않아 재비행 필요로 판단, 즉시 재비행 시작.
`run_results/run_velocity_4m_5mps_monitored.sh attempt1` 사용(이전 세션이
이미 만들어둔, 마지막 웨이포인트 도달 후 STALL 오판 버그가 수정된 스크립트).
동일 조건: flight:=velocity, Seongdong_gu_100x100_dynamic(velocity 모드 자동
선택), path_100x100_5m_4m_5mps.yaml, cruise_speed_mps:=5.0.

**모니터링 중 발견한 버그(스크립트 자체는 아니고 내 감시 루프)**: 이전 세션의
동일 tag(attempt1) 결과 파일 `velocity_4m_5mps_attempt1_result.txt`(04:57에
COMPLETED로 stale하게 남아있던 것)이 지워지지 않은 채 남아있어서, 내가 처음
짠 "결과 파일 존재하면 완료"로 보는 감시 루프가 즉시(1초 만에) 오탐 완료로
잘못 판단했다. 실제로는 gz sim/ros2 bag record가 정상 기동 중이었음(ps로 확인).
stale 파일 삭제 후 재모니터링해서 바로잡음 — 앞으로 같은 tag 재사용 시 이
파일이 남아있을 수 있다는 점 주의.

**중간 진행 상황(재비행)**: 200s 시점 wp 404/5251, bag 1.8GB, TF 정상(~49Hz),
디스크 50%. 순항중.

**사용자(코디네이터 경유) 정책 갱신**:
- 디스크 부족 시 `run_results/clouds/`(6.5GB, 84m/5m 실험 point cloud 원본,
  이미 FN% JSON으로 요약됨)와 `run_results/dryrun_4m_5mps_grid.npz`(90MB)는
  지워도 된다고 명시적 허가받음 — 지운 순서/이유는 실제로 지울 때 여기 기록.
- **정책 전환(중요)**: 이번에 새로 기록하는 bag(velocity_4m_5mps_attemptN,
  최종 성공분)은 실험 종료 후에도 삭제하지 않는다 — 이전 세션들이 "재현
  가능"을 이유로 84m/5m bag을 지운 판단이 바로 이번 재비행을 유발했기
  때문. 실패/중단된 attempt bag만 삭제 대상.

**1~2단계(코드)**: brian_test의 Module A(`drone_elevation_mapper.py`)를
`git show`로 확인 — README에 명시된 요구사항(elevation/variance 칼만필터,
R=거리+입사각+밀도, Q=0, 이노베이션 게이팅 9.0, NaN 패딩, elevation_variance
레이어, **np.gradient 로컬 윈도우 최적화**) 전부 실제로 존재함을 확인(문제
없음, 별도 보고 불필요). 이걸 test_main_brian의 같은 파일에 이식하면서:
- 이 브랜치 고유의 `min_range_m`(기체 자기 반사 제거, transform 전 센서
  로컬 좌표 원점 기준 거리 필터) 유지.
- Module D(`ground_elevation_mapper.py`, brian_test)의 방어 로직 2가지를
  참고해 Module A에 새로 구현: `max_sensor_range`(200.0, transform 후
  센서 원점 기준 거리 필터, `_accumulate`에서 min_range 다음/그리드
  비닝 전) + `max_grid_cells`(30,000,000, `_grow_to_fit`에서 패딩 직전
  예상 총 셀 수 계산 후 초과 시 (None, None) 반환 → `_accumulate`에서
  이번 배치만 버리고 기존 누적 보존).
- `drone_elevation_mapper.yaml`에 칼만필터 파라미터 + 두 방어 파라미터
  전부 추가.
- `agconav_drone/package.xml`의 `<depend>rosbag2_py</depend>` 바로 다음
  줄에 `<exec_depend>rosbag2_storage_mcap</exec_depend>` 추가.

**다음**: 재비행 완료 대기(메모리가 빠듯함 - 277MB free, swap 1GB 사용 중이라
colcon build는 비행 완전 종료 후에 실행해 자원 경합 방지). 완료 후 build →
bag 재생(module_a 라이브 구독) → 채점 → 비교표.

**타임아웃 변경(사용자 지시, 코디네이터 경유)**: 재비행이 필요해져 전체
작업 타임아웃을 2시간 → 5시간으로 연장(시작 시각 기준, 이미 지난 시간
포함). 재비행 자체의 정지조건(3~4회 재시도 불안정, TF 끊김, 디스크 80%
하드컷)은 그대로 유지하되, 디스크 70%부터는 선제적으로
`run_results/clouds/`, `dryrun_4m_5mps_grid.npz` 삭제로 여유를 확보하는
정책을 추가 적용한다(80%가 스크립트의 하드 컷이므로 70%는 그 전에 손 쓸
여유를 두기 위한 조기경보 기준).

**선제적 디스크 정리(70% 조기경보, 사용자 사전 허가)**: 재비행 진행 중
디스크가 60%를 넘고 증가 추세(비행 완료 시점 예상 ~74%)로 보여, 완료 후
build+replay에서 추가로 필요할 여유를 확보하기 위해 미리 정리:
- `run_results/clouds/`(6.5GB, 84m/5m 실험의 원본 point cloud .npy) — 이미
  FN% JSON(`84m_*_fn.json`, `*_4m_5mps_fn.json`)으로 요약 완료, 필요시
  build_methodB_cloud.py로 재생성 가능해 원본 보존 불필요.
- `run_results/dryrun_4m_5mps_grid.npz`(90MB) — 2단계 드라이런 캐시, 이미
  분석 완료.
삭제 후 디스크 사용량 재확인함(로그 참고).

## 3단계 진행 — bag 재생 시작 (칼만필터 + 방어 로직 2겹 라이브 검증)

**재비행 완주**: attempt1, path_status=true, 2231s(약 37분), bag 22GB(12개
mcap 청크), TF/프로세스 전부 정상 종료. `bags/velocity_4m_5mps_attempt1`
보존(정책대로 삭제하지 않음).

**colcon build**: `colcon build --symlink-install --packages-up-to
agconav_drone agconav_traversability` — 성공, 에러 없음(deprecation 경고만).

**bag 재생 실행**: `run_results/run_bag_replay_kalman.sh
bags/velocity_4m_5mps_attempt1 kalman_4m_5mps
run_results/kalman_4m_5mps_fn.json 92 1.0 3600` — drone_elevation_mapper,
terrain_feature_calculator, traversability_verdictor(wheel/leg),
elevation_map_saver 5개 노드를 use_sim_time:=true로 먼저 띄운 뒤
`ros2 bag play --clock --rate 1.0`으로 원본 /drone/points, /tf, /tf_static,
/drone/path_status를 재생. capture_nav_fn.py가 최종 산출물을 구독해 채점.

**이상치 방어 로그 확인(3단계 요구사항)**: drone_elevation_mapper 시작 로그에
`max_sensor_range=200.0m, max_grid_cells=30000000, min_range_m=2.5m`가 정확히
찍히는 것을 확인 — 파라미터가 정상 선언/초기화됨(실제 컷오프 동작 여부는
capture_nav_fn.py 결과의 coverage/height 통계로 간접 확인 예정 — 200m/
3천만 셀 둘 다 이번 100x100 지도 규모에서는 정상 상황에선 거의 안 걸릴
임계값이라 트리거 자체가 로그에 안 남는 게 오히려 정상, 콜백 경로가
빠짐없이 실행됐다는 것만 확인).

**다음**: bag 재생 완료(예상 ~37분, --rate 1.0) 대기 → wheel/leg FN% 채점
→ 비교표 갱신 → git commit/push.

**알려진 버그(이번에 발견) — SIGTERM 에스컬레이션이 bag record를 강제종료시켜
metadata.yaml 누락**: `run_velocity_4m_5mps_monitored.sh`의
`shutdown_launch()`가 launch 프로세스에 SIGINT→(15초 내 안 죽으면)
SIGTERM으로 에스컬레이션하는데, 이번 정상 완주 종료 과정에서 실제로
SIGTERM까지 에스컬레이션됐다(status_log: "SIGINT로 안 죽어서 SIGTERM
에스컬레이션"). 그 여파로 하위 `ros2 bag record` 프로세스가 정상적인
graceful shutdown(각 mcap 청크의 요약/인덱스 섹션 flush + metadata.yaml
작성)을 못 마치고 죽어, bag 디렉토리에 mcap 파일 12개(22GB, 데이터 자체는
전부 정상)는 있는데 `metadata.yaml`이 없는 상태가 됐다. 그 결과
`ros2 bag play`가 "No storage id specified, and no plugin found that could
open URI" 에러로 즉시 실패.

**복구**: `ros2 bag reindex -s mcap bags/velocity_4m_5mps_attempt1` 실행 →
성공("Could not set read order on open(), falling back to file order" 경고는
있었지만 — mcap 파일 자체에 메시지 인덱스가 없어 수신 순서 재정렬을 못하고
파일에 쓰인 순서를 그대로 쓴다는 뜻일 뿐, recorder가 어차피 시간순으로
기록하므로 실질적 영향 없음). `ros2 bag info`로 데이터 무결성 확인:
duration 2258.8s, `/drone/points` 22,319개, `/tf` 111,598개, `/tf_static` 1개,
`/drone/path_status` 1개, `/drone/imu` 223,053개 — 데이터 손실 없음.

**재발 방지**: 이 스크립트류(`run_velocity_*_monitored.sh`,
`shutdown_launch` 패턴을 쓰는 모든 것)로 기록한 bag은, 재생하기 전에 항상
`ros2 bag info <bag_dir>`로 metadata.yaml 존재/무결성을 먼저 확인하는
습관을 들인다(이번 세션의 이후 단계부터 바로 적용). 근본 수정(SIGINT
타임아웃을 늘리거나 bag record만 먼저 별도로 SIGINT하는 등)은 이번 작업
범위 밖이라 이번엔 안 건드리고, 발생 시 reindex로 복구하는 절차만 확립.

## 4~6단계 완료 — 채점 결과 및 최종 비교표

**4단계 채점 결과**(`run_results/kalman_4m_5mps_fn.json`):
- wheel FN% = 10.91% (n_gt_traversable=618,378, occupied_FN=67,436, unknown=9,046)
- leg FN% = 8.03% (occupied_FN=49,646, unknown 동일 9,046)
- elevation_map coverage = 95.80%(997,427/1,041,148), height 1.04~9.04m
- step(단차) 중앙값 0.00692m, p95 0.953m, max 6.519m — baseline(3번)보다
  전부 개선.

**5단계 비교표(최종)**은 `run_results/SUMMARY.md` 최상단 절 참조(5개 행
전체, 분석 포함) — 여기서는 요약만:

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% |
|---|---|---|---|---|---|---|---|
| 1 | 84m | 8m/s | 32m | GT만 | 단순평균 | 23.94% | 12.15% |
| 2 | 84m | 8m/s | 32m | GT+GICP | GICP정합 | 20.10% | 11.89% |
| 3 | 5m AGL | 5m/s | 4m | GT만 | 단순평균 | 13.44% | 12.60% |
| 4 | 5m AGL | 5m/s | 4m | GT+GICP | GICP정합 | 37.26% | 30.03% |
| 5 | 5m AGL | 5m/s | 4m | GT만 | 칼만필터(이상치방어) | **10.91%** | **8.03%** |

핵심 결론: 칼만필터(5번)는 3번(같은 GT-only, 알고리즘만 다름) 대비
wheel -18.8%/leg -36.3%(상대) 개선, 4번(GICP) 대비는 wheel 3.4배/leg
3.7배 개선. 원시 노이즈(step 중앙값/p95/max), 커버리지(95.80%),
미측정비율(1.46%) 전부 3번보다 개선 — GICP(4번)처럼 "쉬운 곳만 쉽게"가
아니라 지도 전체 품질을 실제로 높였다. 4번(GICP) 대비 압도적 우위는
사용자가 제시한 가설(강체변환 전체 오염 vs 셀단위 독립 처리)과 실측이
정확히 일치(4번은 자기 baseline인 3번보다 step 중앙값 5.3배 악화, 5번은
같은 3번보다 24.5% 개선 — 정반대 방향). 상세 근거는 SUMMARY.md 참조.

**6단계 — 저장/커밋**: 아래 커밋 로그 참조.

## 이번 세션 최종 상태

전체 6단계(0~6) 완료. 정지조건 발동 없음. bag(`bags/velocity_4m_5mps_
attempt1`, 22GB)은 정책대로 보존(삭제 안 함, .gitignore로 git 추적 제외).
다음 세션 이어받을 지점: 없음(이번 작업 범위 완료). 참고로 남길 것 —
84m 조건에서의 칼만필터 실측은 아직 없음(SUMMARY.md "다음 결정 지점"
참조, 이번 범위 밖이라 미수행).

## [새 세션] R보정(①)+디스큐(④) 변형 A/B/C 작업 시작

**0단계**: #5 실험 bag(`bags/velocity_4m_5mps_attempt1`, 22GB) `ros2 bag info`로 무결성
재확인 완료(duration 2258.8s, /drone/points 22,319개, /tf 111,598개, /drone/path_status
1개) — **재비행 불필요**, 이 bag을 3단계 재처리에 그대로 재사용.

**디스크 정리 정책(코디네이터 경유, 사용자 지시)**:
- 절대 보호: `bags/velocity_4m_5mps_attempt1`(GitHub에 없음, 삭제 시 재비행 필요),
  `run_results/PROGRESS.md`/`SUMMARY.md`(append만), `path_100x100_5m_4m_5mps.yaml`.
- 부족 시 삭제 후보: 이미 push된 커밋에 결과가 기록된 이전 실험의 원본
  bag/중간산출물(clouds/ 등), 이미 채점 완료된 GICP 중간 캐시, 안 쓰는 Docker 이미지.
- 지우기 전 반드시 해당 결과가 원격(origin/test_main_brian)에 이미 있는지 확인하고
  커밋 해시와 함께 여기 기록할 것.

**세션 크래시 → 재개(이번 세션)**: `pgrep`으로 잔여 ros2/gz/docker 프로세스 없음
확인(깨끗하게 죽어있었음). `bags/velocity_4m_5mps_attempt1` 재확인 완료(위 0단계
기록 그대로 살아있음). `run_results/calib_summary.json`에 `--pilot`(조건 1개,
10스캔) 결과 1건만 있고 나머지 11개 미수집 — ①번은 부분 진행 상태로 판단, 이어서
진행.

**버그 발견 및 수정 — teleport 유실**: 기존 `--pilot` 결과를 검증차 재현하려고
`diag_probe.py`(신규, 1회성 진단용)로 원시 로컬프레임 점까지 덤프해보니, 커맨드한
목표(-25,-100,6.4177)와 실제 TF상 base_link 위치가 전혀 다름(-12.2,-91.1,84.0 —
성분으로 봐서 이전 84m 실험 때 월드에 남은 기본 스폰 pose로 추정)을 발견. `ros2
service call .../set_pose`로 직접 호출하면 즉시 정확히 이동하는데, `/drone/cmd_pose`
토픽 경유(캘리브 스크립트가 쓰는 방식)로는 노드 시작 직후 첫 호출에서 씹혔다 —
`drone_pose_controller` 구독자와의 DDS 디스커버리가 아직 안 끝난 상태에서 publish가
유실되는 전형적인 레이스. 재현: 같은 프로세스에서 시간을 더 준 뒤 두 번째로
`cmd_pose`를 통해 teleport하니 정상 도착(map z within 0.5m: n=222, height_error
~0±0.007m — 매우 깨끗). 즉 **기존 pilot 결과(range_mean 3.94, height_error_mean
0.998±0.969m)는 진짜 센서 특성이 아니라 이 레이스로 드론이 엉뚱한 곳(84m 상공)을
찍은 오염된 데이터** — 폐기.
- **수정**: `calibrate_noise.py`의 `teleport()`에 TF 기반 도착 확인(허용오차
  0.1m, 최대 10초 재시도)을 추가, 확인 전까지 스캔 수집 시작 안 함. 수정 후
  `--pilot` 재실행 결과: range_mean=4.84(≈5m-라이다오프셋0.175, 이론과 일치),
  incidence_mean=3.91°(0도 조건에 맞게 거의 수직), height_error_mean=-0.0001m
  ±0.0068m(P0 실측 평탄도 std=0.0064m와 거의 동일) — 정상 확인.
- 이 레이스는 **각 조건의 첫 teleport에서만** 위험하다고 판단(디스커버리는
  프로세스당 1회성이라 한 번 매칭되면 이후 재사용됨)는 확신이 없어, 수정을 모든
  teleport 호출에 적용(조건마다 재확인) — 방어적으로 안전한 선택.

## ①번 12조건 본 실행 완료

`calibrate_noise.py --out run_results/calib_summary.json` 1차 실행: 12개 중
6개(거리4×입사각0°/0.5m반경 4개 + 5m/60°,75° 2개)는 즉시 성공(50스캔),
나머지 6개(20/50/84m × 60°/75°)는 반경 0.5m 안에 점이 0개(`NO_POINTS_NEAR_TARGET`)
— 거리·입사각이 커질수록 빔 간격(footprint)이 넓어지는 정상적인 기하 현상으로
판단(결정론적 센서라 스캔을 늘려도 해결 안 됨). `RADIUS_LADDER`(0.5→1→2→3m
단계적 확대, target_radius_used_m로 기록) + `--merge`(기존 성공분 재사용, 실패분만
재시도) 기능을 스크립트에 추가해 재실행 → 20/60·50/60·84/60은 반경 2m에서,
84/75는 반경 3m에서 성공. 단 84/75는 height_error_mean=2.17±0.66m로 P0
평탄부가 아닌 다른 지형/물체가 반경 안에 섞여 들어간 것으로 판단해 **폐기**.
20/75·50/75는 반경 3m까지도 0점 — **측정 불가로 처리**(모델 피팅에서 제외).
상세 표·분석·최종 R 모델은 `run_results/calibrated_noise_table.md` 참고.

**핵심 결론**: 실측 σ는 거리 5~84m·입사각 0~75° 전 구간에서 0.006~0.009m로
거의 평탄 — 기존 이론표(0.007→0.050m, `1/cos²θ`)가 예측하는 거리·입사각
의존성이 이 무노이즈 결정론적 시뮬레이터에는 거의 없다시피 함(양자화가
지배적 오차원). **새 R 모델**: `measurement_noise_max_distances=[20,50,90,170]`,
`measurement_noise_sigmas=[0.008,0.009,0.010,0.012]`,
`measurement_noise_incidence_exponent=0.3`(기존 하드코딩 `**2`를 파라미터화 —
아래 코드 변경 참고).

## 코드 변경 (①④, 이번 세션 범위 내)

`src/agconav_drone/agconav_drone/drone_elevation_mapper.py` +
`config/drone_elevation_mapper.yaml`:
1. **`measurement_noise_incidence_exponent`** 신규 파라미터(기본 2.0 — 기존
   `r_point = r_distance / cos_theta ** 2` 동작과 완전히 동일하게 유지, 기준(#5)/
   변형B는 이 기본값을 그대로 씀). `_measurement_noise`가 하드코딩된 `** 2`
   대신 이 파라미터를 쓰도록 수정.
2. **`deskew_enabled`/`deskew_num_buckets`(기본 12)/`deskew_scan_period_sec`
   (기본 0.1)** 신규 파라미터(기본 deskew_enabled=False — 기존 동작 유지).
   `_points_callback`이 deskew_enabled일 때 새 메서드 `_accumulate_deskewed`로
   분기: `/drone/points`가 점별 타임스탬프 필드가 없음을 실측 확인했으므로
   (필드: x,y,z,intensity,ring뿐), organized cloud(height=32 ring x width=1024
   azimuth 발사순서)의 **column index를 그대로 스캔 내 상대시각 근사**에
   씀(`t(col)=header.stamp+(col/width)*scan_period`, atan2 방위각 계산 불필요 —
   column이 이미 발사 순서). 스캔을 12구간(버킷)으로 나눠 구간별 대표시각으로
   TF를 개별 조회, 그 구간 점들만 그 TF로 변환. 기존 `_accumulate`의 후반부
   (사거리 상한 필터~칼만 갱신)는 `_accumulate_common`으로 추출해 두 경로가
   공유 — 중복 없이 안전하게 재사용.
   - 성능: 콜백당 TF 조회가 1회→최대 12회로 늘지만 각 조회는 O(log n) 버퍼
     탐색이라 가볍고, 스캔 자체가 ~10Hz(bag 기준 22319 msg/2258.8s≈9.9Hz)라
     최대 ~119회/초 — `np.gradient 전체 배열` 때 겪은 것과 같은 스케일
     문제는 아니라고 판단(별도 프로파일링은 안 함, 재처리 시 콜백 지연이
     관측되면 버킷 수를 줄이는 것으로 대응 가능하게 파라미터화해둠).
   - R 계산(`_measurement_noise`)의 sensor_origin은 스캔의 마지막 구간
     위치로 근사(정밀 근거는 코드 주석 참고 — 스캔당 이동거리가 R
     구간표 폭보다 훨씬 작아 영향 무시 가능). 점 좌표 자체(디스큐의 본래
     목적)는 구간별로 정확히 변환됨.
3. `run_results/calibrate_noise.py`의 `teleport()` — 위 "버그 발견 및 수정"
   절 참고.

빌드: ament_python symlink-install이라 소스 수정이 install/에 바로 반영됨
(재빌드 불필요, `install/.../drone_elevation_mapper`가 symlink임을 확인).
`python3 -m py_compile`로 문법 확인만 완료.

## 3단계 — A/B/C 재처리 시작

`run_results/run_variant_replay.sh`(`run_bag_replay_kalman.sh`를 일반화 —
drone_elevation_mapper에 주는 추가 `-p` 오버라이드만 변형별로 다르고 나머지
(F/verdictor/saver/capture_nav_fn.py)는 #5 베이스라인과 완전히 동일하게 유지,
공정 비교를 위해 의도적으로 그대로 둠)로 bag(`bags/velocity_4m_5mps_attempt1`,
2258.8s) 재생 + `--clock`, rate=1.0(#5와 동일 — 재현성 위해 속도 변경 안 함).
1회 재생에 ~38분 소요 예상, A/B/C 순차 실행(동시 실행은 5.8GiB RAM에 부담 —
이전 세션이 "66개+ 프로세스 동시 기동"으로 스왑 진입한 전례 참고, 순차가
안전).

- **변형A(calibR만, 디스큐 없음)** 완료: `run_results/variantA_calibR_fn.json`.
  1차 실행은 `_accumulate_common`에 리팩터링 중 남긴 버그(`msg` 미정의
  NameError, 매 콜백마다 크래시)로 즉시 실패 → 짧은 bag 슬라이스로
  스모크테스트해 재현·수정(`_accumulate_common`이 `msg` 대신 `stamp`를
  별도 인자로 받게 변경, 호출부 2곳 수정) 확인 후 재실행, 정상 완주.

  **결과가 기준(#5) 대비 크게 악화됨** — wheel FN% 10.91%→**40.18%**,
  leg FN% 8.03%→**25.48%**, step.median 0.0069→0.0736(10배),
  step.wheel_exceed 12.0%→47.7%. 버그가 아니라 **①과 ④의 상호작용으로
  해석**: ①의 R 실측은 드론이 완전히 정지한 teleport 조건에서만 쟀는데,
  #5 bag은 5m/s 등속비행이라 스캔 0.1초 동안 드론이 이동하면서 생기는
  "모션 블러"(디스큐가 원래 없애야 할 바로 그 왜곡)가 실제 데이터에는
  섞여 있다. 기존(이론 기반) R 표의 큰 σ값이 이 모션 블러를 우연히
  흡수/완충해주고 있었는데, ①만 적용해 σ를 실측(정지 상태 기준)값으로
  대폭 줄이자 칼만필터가 모션 블러로 흩어진 점들을 "믿을 만한 새 측정"으로
  과신하게 되어 지도가 오히려 훨씬 거칠어진 것으로 판단. **①은 ④ 없이
  단독 적용하면 역효과 — 다음 변형B/C 결과로 이 가설을 검증한다.**
- **변형B(디스큐만, 기존 R표)** 완료: `run_results/variantB_deskew_fn.json`.
  wheel FN%=15.00%(기준 대비 +4.09pp 악화), leg FN%=7.53%(기준 대비 -0.50pp
  개선) — wheel/leg가 반대 방향으로 갈렸다. 동시에 `unknown_percent`가
  1.46%(기준)→9.47%로 급증(coverage_percent도 95.80%→92.95%) — 디스큐가
  버킷(12구간)마다 개별 TF를 조회하다 보니, 특히 스캔 끝쪽 구간(방위각
  360°에 가까운 col)이 요구하는 시각이 그 시점 TF 버퍼의 최신 샘플보다
  살짝 앞서(extrapolation-into-future) 조회 실패하는 경우가 실전(1x 재생)
  에서도 체계적으로 발생하는 것으로 추정(스모크테스트 때 5x 재생에서
  이미 같은 유형의 경고를 실측함, top-of-file 참고) — 그 구간 점들이
  통째로 버려져(`_accumulate_deskewed`의 "구간만 버림" 설계) 커버리지가
  줄어든 것으로 해석. DEBUG 레벨 로그라 기본 실행에서는 빈도를 직접 세지
  못했음(정밀 카운트는 이번 범위 밖으로 생략, 아래 최종 분석에서 이
  한계를 명시).
- **변형C(①+④ 둘 다)** 완료: `run_results/variantC_calibR_deskew_fn.json`.
  wheel FN%=27.44%, leg FN%=14.48% — A(①만, 40.18/25.48)보다는 크게
  낫지만 B(④만, 15.00/7.53)보다는 나쁘고, 기준(10.91/8.03)에는 한참
  못 미침. unknown_percent는 B와 거의 동일(9.35% — 디스큐발 커버리지
  손실이 그대로 남음, 이는 예상대로 ①이 아니라 ④의 부작용이라는 근거).

## 5단계 — 비교표 및 분석

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% | 비고 |
|---|---|---|---|---|---|---|---|---|
| 5 | 5m AGL | 5m/s | 4m | GT만 | 칼만필터(이상치방어) | **10.91%** | **8.03%** | 기준 |
| A | 5m AGL | 5m/s | 4m | GT만 | +①실측R보정 | **40.18%** | **25.48%** | ① 단독 심각한 악화 |
| B | 5m AGL | 5m/s | 4m | GT만 | +④디스큐 | **15.00%** | **7.53%** | ④ 단독, wheel 악화·leg 소폭 개선 |
| C | 5m AGL | 5m/s | 4m | GT만 | +①+④ | **27.44%** | **14.48%** | 둘 다, 기준 대비 여전히 악화 |

**①(R보정) 개별 기여**: wheel +29.27pp(+268% 상대), leg +17.45pp(+217% 상대)
— 12조건 teleport(완전 정지) 실측을 5m/s 등속 비행 데이터에 그대로
적용한 것이 근본 원인으로 판단(아래 참고).

**④(디스큐) 개별 기여**: wheel +4.09pp(+37.5% 상대, 악화), leg -0.50pp
(-6.2% 상대, 개선). 방향이 엇갈리고 크기도 작아 이 실험 조건에서는
효과가 뚜렷하지 않음 — 위에서 짚은 구현상 커버리지 손실(unknown
+8pp)이 잠재적 이득을 상쇄했을 가능성.

**둘을 합쳤을 때(C) — 단순 합산도 시너지도 아니라 "부분 상쇄"**:
퍼센트포인트 기준 단순 합산 예측은 wheel 10.91+29.27+4.09=44.27,
leg 8.03+17.45-0.50=24.98인데, 실측 C는 wheel 27.44·leg 14.48로
훨씬 낮다 — **④가 ①의 손상을 상당 부분(하지만 완전하지는 않게) 상쇄**
하는 것으로 해석된다. 이는 최초 가설("①은 정지 상태에서만 잰 실측이라
등속비행의 모션 블러를 반영 못 하고, ④가 그 블러를 없애야 ①이 제 역할을
한다")과 방향이 일치 — ④를 더해주자 ①의 피해가 40%→27%(wheel),
25%→14%(leg)로 줄어들었다. 다만 완전히 상쇄되지 않고 기준보다도 여전히
훨씬 나쁘다: (1) ④ 자체가 도입한 커버리지 손실(위 참고)이 순이익을
깎아먹고, (2) 12버킷 근사가 실제 모션 블러를 완벽히 제거하지 못했을
가능성, (3) ①의 실측 σ 자체가 애초에 정지 조건 기준이라 비행 중
진짜 유효 노이즈(모션블러+양자화+미세지형)보다 구조적으로 작게 잡혔을
가능성이 남아있다.

**결론 및 권고**: 이번 실험 조건(5m AGL, 4m 간격, 5m/s)에서는 **① 단독,
④ 단독, ①+④ 모두 기준(#5, 이론 기반 R표+칼만필터+이상치방어)보다
나쁘다** — 셋 다 채택 비권고. ①의 실측 R 표(calibrated_noise_table.md)
자체는 "이 시뮬레이터의 정지 상태 센서 특성"으로는 유효하지만, 이동
플랫폼 데이터에 안전하게 쓰려면 ④가 완전히 신뢰할 수 있는 수준까지
개선돼야 한다(현재 구현은 커버리지를 깎는 부작용이 있음). 기존
이론 기반 R표의 상대적으로 큰 σ가 정지-비행 간극에 대한 암묵적
안전마진 역할을 하고 있었다는 것이 이번 실험의 핵심 발견이다.

## 6단계 — 저장/보고 완료, 세션 종료

- `git add`(run_results 신규/수정분 + drone_elevation_mapper.py/.yaml) →
  커밋 `4ef9a0c` → `git push origin test_main_brian` 완료(원격 최신).
  `src/agconav_test_worlds/config/path_calib_dummy.yaml`(이전 크래시
  세션이 남긴, 이미 폐기된 P0 후보(-20.241,-112.194) 탐색용 미커밋
  파일)은 이번 작업과 무관해 커밋하지 않고 그대로 둠(삭제도 안 함 —
  범위 밖 정리).
- 프로세스: ros2/gz sim/docker 관련 잔여 프로세스 없음 확인(dockerd
  데몬 자체는 정상). 디스크 69%(24G 여유) — 안전.
- 소요 시간: 세션 시작~완료 약 2시간 40분 (① 버그 조사·수정 포함).
  타임아웃(3시간) 안에 6단계 전부 완료.

### 세션 재개용 참고 (다음 세션이 필요할 경우)
이번 세션의 모든 단계(0~6)가 끝까지 정상 진행됨 — 다음 세션이 이어받을
미완료 작업은 없다. 후속으로 고려할 수 있는 것(이번 범위 밖, 미수행):
- ④ 디스큐의 버킷별 TF extrapolation 실패로 인한 커버리지 손실(unknown
  +8pp) 원인을 DEBUG 로그 레벨을 올려 정밀 계측하고, 버킷 대표시각을
  "구간 중앙"이 아니라 "구간 시작 직후"로 당기는 등으로 완화 가능한지 검토.
- ①의 실측을 "완전 정지"가 아니라 등속 이동 중(디스큐 적용 상태)에
  직접 재실측해서, 진짜 유효 노이즈(모션블러 제거 후 잔차)를 구하는 방법
  — 이번엔 지시사항대로 teleport(정지)만 썼지만, 결과가 시사하는 바로는
  이 접근이 더 정확한 R을 줄 가능성이 있음.

## 세션 재개용 참고 (다음 세션이 끊긴 채로 이어받을 경우)
- `pgrep -af "drone_elevation_mapper|ros2 bag play|traversability_verdictor"`로
  현재 어느 변형이 돌고 있는지 확인. `run_results/variant{A,B,C}_*_fn.json`
  중 존재하는 파일로 어디까지 끝났는지 판단.
- gz sim/Gazebo는 이 3단계(bag 재처리)에는 필요 없다(bag play --clock이
  유일한 시간 소스) — 혹시 남아있으면 관계없는 잔재이니 정리.

## [새 세션] GLIM odometry 발산 원인 진단 (0단계 glim_ext + 1단계 진단, 파라미터 튜닝 없음)

**전체 타임아웃 3시간.** 판단 지점은 확인 없이 스스로 진행, 근거는 이
문서에 계속 기록. 목표는 직전 세션(위 "[새 세션] GLIM(+GPS 사후결합)
전체 파이프라인 실험" 절)이 미완료로 남긴 "GLIM 궤적이 4가지 조합
전부에서 발산했다"는 문제의 **원인 진단**(파라미터 튜닝은 다음 세션/
Phase 2로 미룸).

### -1. 재개 확인
`pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음(깨끗). `df -h ~`
→ 72%(21G 여유) — 임계값(70%) 초과라 정리 필요. `git log` HEAD가
직전 세션 커밋(`5a8559d`)과 일치. **GPS 포함 velocity bag
(`bags/velocity_4m_5mps_gps_attempt2`, 23G) 존재 확인, 재사용
가능 — 재비행 불필요.**

### -0.5. 디스크 정리
1순위 항목(84m/GICP 실험 원본 bag 등)은 이미 이전 세션에서 정리돼
남아있지 않았다(확인만 하고 스킵). 대신 `bags/_chunk_t`(2.1G, 직전
세션이 청크 처리 중 중단하며 남긴 스트레이 중간 산출물 — 원본이 아니라
`velocity_4m_5mps_gps_attempt2_0.mcap`에서 재생성 가능)를 삭제.
`bags/exp_teleport`(144M)는 용도가 불명확해(애매함) 보존. 결과:
72%→70%(23G 여유)로 회복.

### 0단계 — glim_ext 빌드: 성공 (상세는 `run_results/glim_ext_feasibility.md`)
`~/glim_ext_ws`에 clone+colcon build, 1차 시도부터 성공(추가 -dev
패키지 설치 불필요 — apt PPA로 깔린 GLIM/GTSAM 개발 헤더가 이미
충분했음). `libimu_validator.so`/`libgnss_global.so` 정상 빌드,
FAST-LIO2만 서브모듈(SSH 전용 URL)을 못 받아 자동 제외(빌드 자체는
안 깨짐). `config_ros.json`에 `libimu_validator.so`를 등록해 두
번 테스트: (1) glim_ext 워크스페이스 미소싱 → GitHub 이슈 #9와
동일한 "glim_ext package path was not found" + "cannot open shared
object file" 재현 확인, (2) `source ~/glim_ext_ws/install/setup.bash`
추가 → 에러/경고 없이 정상 로드 확인. 라이선스: GPLv3(top-level
`package.xml`, `gnss_global`도 동일) — "closed-source"는 아니고
README 경고는 "미완성/유지보수 부족" 취지. 이번 진단(④) 범위엔
라이선스 문제 없음.

### 1단계 — 원인 진단
`glim_config/`(기존 그대로, CT+passthrough+pose_graph)로 GLIM을
실행하되, **전체 bag(2254s) 대신 chunk 0(190.5s, t=8.9~198s)만
사용**했다 — 직전 세션이 이미 4가지 조합 전부에서 t≈101~136s
부근부터 "물리적으로 불가능한" 절대값으로 발산함을 실측해뒀고,
chunk 0이 이 구간을 충분히 포함해 전체(처리에 5~6시간 필요, 직전
세션 실측)를 다시 돌릴 필요가 없다고 판단했다.

- `add_point_times_single.py`로 chunk 0에 point-level `t` 필드
  추가(`bags/_chunk_t`, 처리 후 삭제 — 디스크에 남기지 않음).
- `ros2 run glim_ros glim_rosnode --ros-args -p config_path:=...`로
  라이브 실행(메모리 가드 4600MB 적용, 이번엔 chunk가 짧아 발동
  안 하고 정상 종료) + `ros2 bag play bags/_chunk_t --clock --rate
  1.0`(190.5s 실시간 재생) → SIGINT(실제 노드 PID, `ros2 run` 래퍼
  PID가 아님 — 처음에 래퍼 PID에 보내서 반응 없었던 것 확인 후 수정)
  → `/tmp/dump`에 `odom_imu.txt` 등 4종 저장 → 즉시
  `run_results/glim_diag_dumps/run1_full_pipeline/`로 복사(다음
  실행이 `/tmp/dump`를 덮어쓰기 전에).
- **③ 타임스탬프 동기화**: `run_results/diag_scripts/check_timestamps.py`
  로 bag 전체(22508+224866개 메시지) 검사 — 역전 0건, 간격 이상
  14건 전부 t+593s 이후(발산구간과 무관), 발산구간 근처 이상
  0건. **원인에서 배제.**
- **④ LiDAR-IMU 외부보정**: `agconav_test_worlds/models/
  agconav_drone_dynamic/model.sdf`(SDF 1.6, `relative_to` 미사용
  → 모든 `<pose>`가 모델 프레임 기준)를 직접 읽고 T_lidar_imu를
  손으로 재계산(Ry(-90°) 회전 적용) → `config_sensors.json`의
  값과 translation/quaternion 모두 소수점까지 일치 확인.
  imu_validator는 빌드는 됐지만 이 특정 bag으로 실제 구동해보는
  것까지는 안 함(수치 재계산이 더 빠르고 직접적이라고 판단, 시간
  절약) — 필요하면 다음 세션에서 추가 가능. **원인에서 배제.**
- **①② GT-GLIM 비교**: `run_results/diag_scripts/compare_gt_glim.py`
  (Umeyama 정합은 `ate_align.py` 로직 재사용, 단 **초반 5초만으로
  정합**한 뒤 전체 궤적에 적용 — 전체 구간으로 정합하면 발산한
  뒷부분이 정합 자체를 오염시키므로). **핵심 발견: 발산은 직전
  세션이 본 t≈101~136s가 아니라 t≈15~17s부터 이미 시작** — 그
  시점이 GT 상 드론이 정지(hover, t=8.9~13.6s 위치 고정)에서
  전진 비행으로 전환되는 순간과 거의 정확히 일치. 이전 세션의
  "t≈101~136s" 판단은 절대좌표값이 육안으로 명백히 이상해 보이는
  시점이었을 뿐, 실제 GT 대비 오차는 그보다 훨씬 일찍부터 커지고
  있었다는 뜻 — **진단이 한 단계 더 정밀해졌다.**
- **⑤ 디스큐 on/off 비교**: `glim_config_nodeskew/`(사본,
  `global_shutter_lidar: true`만 다름)로 같은 chunk 0을 재실행,
  `run_results/glim_diag_dumps/run2_nodeskew/`에 저장. 발산
  **시작 시점은 거의 동일**(15.7s vs 15.9s, 디스큐가 1차 원인은
  아님) — 그러나 발산 **이후 심각도·양상은 뚜렷이 다름**(ON:
  최대오차 306m·혼란스러운 스크리블, OFF: 135m·매끄러운 단일
  드리프트) → 지시사항의 "다르면 원인이 상당히 좁혀진다" 기준으로
  **디스큐는 2차 악화 요인**으로 좁힘.

### 종합 진단 (상세 근거는 `run_results/divergence_diagnosis.md`)
1순위 후보: **CT 오도메트리의 "정지→전진 전환" 처리**
(`config_odometry_ct.json`의 `constant_velocity_inf_scale` 주석이
"이 bag은 이미 8m/s로 순항 중일 때 시작한다"고 전제하는데, 실측 GT는
이 bag에 실제 정지 구간이 있어 전제와 어긋남 — 다른 실험/속도 조건에서
복사된 설정일 가능성). 2순위: per-point 디스큐(column-index 근사)
정확도. 배제: 타임스탬프 동기화, LiDAR-IMU 외부보정.

Phase 2(파라미터 튜닝, 다음 세션) 권고 순서: (1)
`constant_velocity_inf_scale`을 이 bag의 실제 초기 속도 프로파일에
맞게 재검토(1e0/1e2/1e3 정도로 간단 스윕해 발산 시작이 늦춰지는지만
먼저 확인), (2) 디스큐를 일단 끈 상태를 새 기준선으로 삼고 (1)이
안정화된 뒤 디스큐 정밀도를 별도로 개선(두 요인을 동시에 바꾸면
이번처럼 원인 분리가 어려워짐), (3) 그래도 남으면
`smoother_lag`/`max_correspondence_distance` 검토.

### 정리/저장
- `glim_config_nodeskew/`는 재현·비교 참고용으로 보존(작아서 git에
  포함). `bags/_chunk_t`, `/tmp/dump`, 테스트용 `glim_config_extcheck/`
  는 삭제. GLIM/bag play 잔여 프로세스 없음 확인.
- 최종 디스크: 70%(23G 여유) — 안전.
- `run_results/glim_ext_feasibility.md`, `run_results/divergence_diagnosis.md`
  신규 작성. `SUMMARY.md` 최상단에 이번 세션 요약 추가(이전 세션
  요약은 그대로 아래에 보존).

### 세션 재개용 참고 (다음 세션이 이어받을 경우)
- 이번 세션은 진단만 하고 파라미터를 전혀 바꾸지 않았다 —
  `glim_config/`(기존 실험용)는 손대지 않음, `glim_config_nodeskew/`만
  비교용 사본으로 신규 추가됨.
- Phase 2(파라미터 튜닝)는 위 "종합 진단"의 권고 순서대로 시작하면
  된다. `bags/velocity_4m_5mps_gps_attempt2`(GPS 포함)가 그대로
  남아있으니 재비행 불필요, chunk 0 재사용(또는 필요시 다른
  청크로도 검증) 가능.
- `~/glim_ext_ws/`(빌드 완료 상태)가 홈 디렉터리에 남아있음 — Phase 1
  ④를 imu_validator로 직접 재검증하고 싶다면(이번엔 수치 재계산으로만
  검증) `source ~/glim_ext_ws/install/setup.bash` 추가하고
  `config_ros.json`에 `libimu_validator.so` 등록 후 실행하면 된다.

## [새 세션] 순수 GLIM 표준 재현 실험 (2026-08-19)

**목표**: 지금까지 쌓인 모든 커스텀 우회(수동 청크분할, GPS 사후결합,
메모리 가드 수정)를 걷어내고 GLIM 공식 문서가 권장하는 표준 방식
그대로 100x100 지역 2.5D elevation map을 얻는다. 조건(5m AGL/5m/s/4m
간격) 유지, 재비행 없이 기존 GPS 포함 bag 재사용. 정확도 개선용
파라미터 튜닝·GPS결합 없음. 허용 예외는 딱 2개: ①bag 끝 자유낙하
구간 처음부터 제외, ②지도 생성 시 격자 크기 크래시 방지 상한.
전체 타임아웃 3시간, 판단은 사용자 확인 없이 스스로.

### -1/0단계 — 재개 확인 + 디스크 정리
`pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음(깨끗). branch
`test_main_brian` 확인. `df -h ~` → 71%(22G 여유). 우선순위대로 정리:
`bags/_slice40`(453MB, Phase 2 cv 스윕에서 쓰고 SUMMARY.md에 최종
수치까지 이미 기록 완료, GPS bag에서 재생성 가능) 삭제 → 70%(23G
여유)로 회복. `bags/exp_teleport`(144M)는 용도 불명확(이전 세션도
보류) 상태 그대로 보존. 보호 대상(GPS bag 23G, PROGRESS/SUMMARY,
path_100x100_5m_4m_5mps.yaml, apt GLIM, ~/glim_ext_ws) 전부 확인,
손대지 않음.

### 1-2단계 — GLIM 공식 문서 재검증: 기존 CT+passthrough+pose_graph가
표준이 아니었다는 것 확인

WebFetch로 `https://koide3.github.io/glim/{quickstart,parameters}.html`
확인 + 로컬 apt 설치 패키지(`ros-jazzy-glim` 1.2.2, `/opt/ros/jazzy/
share/glim/config/`)의 **미수정 원본 config.json을 직접 diff**로
대조(웹 요약보다 훨씬 신뢰할 수 있는 근거):
- **패키지 기본(진짜 표준) config.json은 GPU 조합**
  (`config_odometry_gpu.json`+`config_sub_mapping_gpu.json`+
  `config_global_mapping_gpu.json`). 이 VM은 `nvidia-smi` 자체가 없어
  GPU 조합은 애초에 실행 불가(이전 세션이 이미 확인해뒀던 사실 재확인).
- **기존 `glim_config/`(CT+passthrough+pose_graph)는 GPU 코드 실행 불가라는
  제약 + "IMU 회전이 안 잡히는 teleport bag" 대응이라는 이유로 이전
  세션이 고른 특수 조합**이었다 — 이번 GPS bag은 실제 IMU 운동이 있는
  velocity 비행이라 그 특수 조합을 쓸 이유가 없다.
- **패키지에는 CT 말고도 진짜 CPU 표준 조합이 세트로 존재한다**:
  `config_odometry_cpu.json`(LiDAR-IMU tight coupling, GICP/VGICP,
  `validate_imu: true`) + `config_sub_mapping_cpu.json`(`enable_imu:
  true`) + `config_global_mapping_cpu.json`(`enable_imu: true,
  enable_optimization: true`, between-factor+implicit loop closure) —
  기존 `glim_config/`에 이 3개 파일이 이미 미수정 원본으로(diff 0)
  들어있었는데도 안 쓰이고 있었다. **GPU 다음으로 우선 시도해야 할
  진짜 표준은 CT가 아니라 이 CPU LiDAR-IMU 조합**이라고 판단, 새
  디렉터리 `glim_config_cpu_std/`를 만들어 이 3개(전부 미수정 원본)로
  config.json을 구성.
- 부수 발견: `glim_config/config_odometry_ct.json`의
  `constant_velocity_inf_scale`이 원본 기본값(1e3)에서 1e0으로
  낮춰져 있었고, 그 근거 주석("이 bag은 이미 8m/s로 순항 중")이 **다른
  (84m/8m/s) 실험에서 복사된 것으로 이번 5m AGL/5m/s bag과 무관** —
  이건 이번 실험 기준으로 "미승인 정확도 튜닝 잔재"라 판단, CT를
  쓸 경우를 대비해 완전히 미수정 원본(1e3)만 쓰는 `glim_config_ct_std/`도
  별도로 만들었다(둘 다 diff 0 확인).
- `config_sensors.json`/`config_preprocess.json`/`config_ros.json`의
  기존 수정분(T_lidar_imu 실측값, 토픽명 `/drone/*`, QoS best_effort,
  distance_far_thresh 200m, downsample 0.3m, GUI 뷰어 제거)은 전부
  "이 시뮬레이터 센서에 맞춘 필수 환경설정"이지 "정확도 개선용 파라미터
  튜닝"이 아니라고 판단해 그대로 유지(모든 신규 config 디렉터리에 공통
  적용). 원본 bag의 `/drone/points`는 point-level 타임스탬프 필드가
  없음(fields: x,y,z,intensity,ring만, 실측 확인) — 이전 진단 세션이
  썼던 "bag에 t필드 주입" 전처리는 이번엔 하지 않음(그것도 커스텀
  우회이자 재비행 없이 원본 bag 그대로 쓰라는 지시와 배치) — GLIM의
  `autoconf_perpoint_times=true`(패키지 기본값과 동일, 커스텀 아님)
  기본 동작에 그대로 맡김. 실행 로그에 "use pseudo per-point
  timestamps based on the order of points" 경고로 확인됨.

### 1-1/1-3/1-4단계 — 짧은 깨끗한 직진 구간 표준 실행 + GT 대조

GT `/tf`로 실측(`gt_velocity_profile.py`): 이 bag은 t=0~약 33s에
이륙+첫 웨이포인트로 진입하는 정지→상승→오버슈트→재정렬 과정을 거치고
(고도가 3m→10.8m까지 오버슈트했다가 8.4m 근처로 정착), **t≈33~93s가
진짜 깨끗한 등속 직진 구간**(y≈-166.1 고정, x가 -29→+62로 일정하게
증가, 첫 코너는 x=61.4 지점, waypoint yaml 계산상 x=61.4 부근 = 실측
GT에서도 t≈97.9s에 x=62.3로 정확히 일치). 이 구간(t=33~90s, 57초)을
"정지→전진 전환 이후 첫 코너 전 깨끗한 직진 구간"으로 채택.

GLIM 공식 배치 도구 `glim_rosbag`(수동 청크분할 아님 — ROS 파라미터
`start_offset`/`playback_duration`/`auto_quit`/`dump_path`가 도구
자체에 내장된 표준 인터페이스)로 이 구간을 CPU-IMU 표준과 CT 표준
양쪽에 각각 1회 실행(파라미터 튜닝 없음), `compare_gt_glim.py`(기존
진단 스크립트, 초반 5초 Umeyama 정합 후 전체 오차 비교)로 GT 대조:

| 설정 | 초반 거동 | 오차 진행 | 최종(t=97~98s) |
|---|---|---|---|
| CPU-IMU 표준(`glim_config_cpu_std`) | 첫 데이터부터 이미 4.2m(정합 직후 발산 시작) | 단조 증가, 회복 없음 | **151.5m** |
| CT 표준(`glim_config_ct_std`, cv=1e3 원본) | t=44.4s에 err=0.05m(거의 완벽)까지 잠깐 좋음 | t≈43~52s부터 흔들리다 t≈62~70s부터 z가 급락(+4→-47m)하며 파국적 발산 | **79.6m** |

실행 로그: CPU-IMU 표준은 `IMU prediction is not good... IMU better
ratios rot=0.47, trans=0.15, vel=0.29`(IMU 결합이 안 쓰느니만 못한
경우가 더 많다는 자체 경고)가 반복 — 이 시뮬레이터의 IMU/LiDAR-IMU
외부보정 조합에서 GLIM의 LiDAR-IMU tight coupling이 구조적으로
불리하다는 증거. CT는 짧게나마 GT와 1~2m 이내로 맞아떨어지는 구간이
있었지만(정합 노이즈 수준의 순간적 일치라 "성공"이라 부르긴 이르다)
30초 안에 파국적으로 무너졌다.

**1-4단계 판정**: 둘 다 GT 대비 1~2m 이내를 유지하지 못하고 수십~백m
단위로 발산 — **"GLIM/설정/환경 자체의 문제" 쪽 증거로 판정.** 커스텀
코드(칼만필터, GICP, R보정, 디스큐 등)는 이번 실행에 전혀 관여하지
않았으므로(원시 GLIM 궤적만 봄), 이 발산은 그런 다운스트림 커스텀
코드와 무관하게 GLIM 자체 궤적 추정 단계에서 이미 발생하는 것으로
확정. CT가 CPU-IMU보다 상대적으로 나아(초반 짧은 구간 일치, 최종오차
비교적 낮음) **2단계(점진적 확장)의 대표 표준 설정으로 CT 표준
(`glim_config_ct_std`)을 채택**하고, CPU-IMU 결과는 대조군으로
기록만 남긴다.

### 2단계 — 점진적 확장

**확장1(t=0~800s, bag 시작부터 여러 코너 포함)**: `glim_rosbag`
(start_offset=0, playback_duration=800, 청크분할 아님 — 도구 자체 파라미터로
단일 실행) 결과 **크래시/OOM 없이 정상 완주**(약 10분 소요, 메모리
최대 RSS 약 1.45GB, 스왑 소폭 사용 후 회복, 5.8GiB 중 여유 4GB대 유지).
GT 대조(`compare_gt_glim.py`): 발산 자동탐지 **t=22.3s**(거의 이륙
직후), 최대오차 **157.0m**(t=784.2s). 로그에 IMU 관련 경고 없음(CT는
LiDAR-only라 CPU-IMU 표준 때 봤던 "IMU better ratio" 문제 자체가
구조적으로 발생하지 않음), time-gap 경고 1건(t=602.22s, 0.18s 간격 —
경미, 치명적이지 않음).

**판단 — "발산"을 정지조건으로 볼지 재검토**: 오차가 크지만(수십~150m대)
이 프로젝트의 이전 Phase 2 세션(SUMMARY.md "GLIM Phase 2 파라미터
튜닝" 절)이 이미 **같은 constant_velocity_inf_scale=1e3**(이번 실험이
"미수정 원본값"으로 다시 채택한 바로 그 값, 우연의 일치가 아니라
Phase 2가 원본값을 실측으로 재확인했던 것) 조합에서 전체 bag을 끝까지
처리해 **"치명적 발산(물리적으로 불가능한 값)은 없고, 각 스트립
왕복마다 5~140m 사이를 진동하는 bounded 오차, 실비행 구간 평균
68.6m"**라는 것을 이미 검증해뒀다. 지금 800s 구간의 최대오차 157m도
같은 자릿수(bounded, 아직 수백~수천m로 폭주하지 않음)로, **OOM이나
"물리적으로 불가능한 값으로의 파국적 발산"이 아니라 "GLIM 표준
설정의 실제 정확도가 원래 나쁘다"는 뜻으로 판정** — 지시사항의 정지조건
("발산 또는 OOM이 나타나면 그 지점에서 멈춰라")은 후자(파국적
발산/크래시)를 가리키는 것으로 해석하고, 전자(경계가 있는 큰 오차)는
"GLIM 표준 방식의 실제 정확도"로서 계속 진행해 끝까지 지도를 만들고
지표에 정직하게 반영하기로 판단했다. 이 판단 근거를 최종 보고서에도
명시한다.

**확장2(전체 bag, 자유낙하 구간 제외 t<=2129.67s)**: 위 판단에 따라
청크분할 없이 단일 `glim_rosbag` 실행으로 바로 진행(`start_offset=0,
playback_duration=2129.67`). 진행 상황은 이 문서 하단에 이어서 기록.

### 3단계 — 2.5D 지도 생성

`run_results/pure_glim_build_map.py`(신규, drone_elevation_mapper.py는
전혀 건드리지 않음) 작성. `_grow_to_fit`은 drone_elevation_mapper.py의
동명 메서드를 읽기 전용으로 참고해 sum/count 버전으로 독립 재구현(칼만
variance 없음, 셀당 단순 평균만). max_grid_cells=30,000,000(Module D/E와
동일값) 상한을 넘는 배치는 저장하지 않고 에러 로그 후 버리는 안전장치2를
동일하게 구현.

**버그 발견·수정(스모크테스트 중)**: GLIM의 map/odom 프레임이 GT(world)
절대좌표와 무관한 자체 로컬 좌표계(첫 스캔 부근을 원점으로 잡음)라는 걸
스모크테스트로 처음 확인 — 정합 없이 그대로 지도를 만들면 평가영역(BOX)과
전혀 안 겹쳐 커버리지가 항상 0%가 나왔다. **초기 1회 프레임 정합**(Umeyama,
스케일 고정, 궤적 초반 5초만 사용 — `compare_gt_glim.py`가 평가에 이미
쓰던 것과 동일 로직을 독립 재구현)을 추가해 해결. 이건 "GPS 사후결합"
(비행 내내 주기적으로 GPS로 재정합)과는 성격이 다르다고 판단했다 —
로컬 SLAM 좌표계를 세계 좌표계에 최초 1회 등록하는 절차는 실제 로봇
배포에도 필요한 최소한의 절차이고, 이후 궤적을 다시 건드리지 않기
때문이다(이 정합 없이는애초에 좌표축 자체가 안 맞아 지도/평가가 성립
불가능 — GLIM "정확도"를 개선하는 게 아니라 좌표계를 맞추는 것).

`glim_config_ct_std` 전체 bag 실행(`run_results/pure_glim_diag/dump_full_ct`,
21,255개 궤적 포즈, t=[8.80,2134.40])의 `traj_lidar.txt`로 지도 생성:
- 원본 `/drone/points` 21,207개 스캔 처리(궤적 범위 밖 스캔 0개), 점
  261,841,364개(2.5m 미만 자기반사 필터 적용, 기존 스크립트들과 동일 기준).
- 결과 격자 2829x1676(≈283m x 168m) — **평가영역(100x100m)보다 훨씬 큼**,
  궤적이 튀어 격자가 실제 필요한 것보다 훨씬 넓게 확장됐다는 뜻(아래
  안전장치2 발동 여부 참고).
- 유효 셀 850,549개(격자 자체 기준 17.94%).
- **안전장치2(max_grid_cells) 발동 안 함**(요청된 최대 4,741,404 < 상한
  30,000,000) — 다만 격자가 평가영역의 ~4.7배까지 커진 것 자체가 궤적
  부정확성의 정황증거로 기록.
- `run_results/pure_glim_map.png` 저장 — 재구성 지도가 대각선 줄무늬
  형태로 평가영역(붉은 점선)과 부분적으로만 겹치고, 절대고도값이 최대
  180m대까지 나타남(5m AGL 비행에서 물리적으로 불가능 — 궤적 오차가
  XY뿐 아니라 Z에도 그대로 나타난다는 시각적 증거).

### 4단계 — 4가지 지표

`run_results/pure_glim_metrics.py`(신규) — 지시사항대로 Module F의 live
`traversability_verdictor` 노드를 띄우지 않고, 이번 지도의 4-이웃 최대
높이차(step)로 주행성 지도(g: 0=free/1=blocked/-1=unknown)를 직접 구성
(wheel 0.08m/leg 0.15m). `inbox`=평가영역 전체 1,000,000셀. 사용자가 준
연결덩어리 코드를 그대로 사용(wheel r=0.55m, leg r=0.40m).

- **비행 소요시간**: bag epoch로 계산한 `/drone/path_status=true` 시각
  (1786906103.279183234) - bag 시작(1786903973.604773595) =
  **2129.674초(35.49분)**. (이전 세션들의 "2254.09/2258.8초"는 자유낙하
  구간까지 포함한 bag 전체 길이였다 — 이번엔 안전장치1로 그 구간을 아예
  처리 대상에서 뺐으므로 지시사항대로 path_status 기준으로 다시 계산.)
- **커버리지**: 40.99%(409,908/1,000,000셀).
- **wheel FN%**: 33.39%(206,475/618,378, 미측정 66.31%). leg FN%:
  33.12%(204,806/618,378, 미측정 66.31%).
- **최대연결덩어리%**: wheel **0.00%**, leg **0.00%** — GT 통과가능 셀
  618,378개 중 우리 지도가 실제 "free"로 옳게 잡은 셀 자체가 wheel
  1,858개/leg 3,527개뿐이라 로봇 몸체 크기 disk로 침식하면 살아남는
  연결영역이 사실상 없다.

전체 결과: `run_results/pure_glim_metrics.json`, `run_results/pure_glim_map.npz`.

### 5단계 — 결론

1단계(다운스트림 커스텀 코드 완전 배제, GLIM 원시 궤적만 GT와 대조)부터
이미 수십~200m 단위로 벗어났고 전체 bag으로 확장해도 같은 bounded-발산
양상이 유지됨 — **"GLIM/설정/환경 자체의 문제"로 확정, 커스텀 코드
문제가 아니다.** 100x100 처리 자체는 크래시/OOM 없이 끝까지 완주했지만
(시간/코너 기준 100%), 궤적 부정확성 때문에 실제 유효 커버리지는
40.99%에 그쳤다 — "일부 구간에서 멈췄다"가 아니라 "끝까지는 갔지만
위치가 많이 틀렸다"는 실패 양상. baseline #5(칼만필터+GT pose, wheel
FN% 10.91%)와 비교하면 wheel FN%가 3배 이상 나쁘고 최대연결덩어리가
사실상 0%다. "GLIM+GPS cv1e3+사후결합" 비교행은 `git log`/PROGRESS.md
전체 검색으로 확인한 결과 과거 세션에서 **실행까지 못 가고 미완료로
남아있어**(GLIM 궤적 발산으로 중단, 커밋 `5a8559d`) 숫자를 지어내지
않고 "미완료"로 표시했다. 상세 비교표·근거는
`run_results/pure_glim_result.md` 참고.

**핵심 결론**: 이전 세션들이 도입한 커스텀 우회(GT pose 신뢰, 칼만필터
이상치방어, GPS 사후결합 등)는 결과를 인위적으로 좋게 포장한 게 아니라,
GLIM 표준 파이프라인이 이 환경(GPU 없는 VM + 이 시뮬레이터의 센서 특성)
에서 스스로 해결하지 못하는 실제 정확도 문제를 메우기 위해 필요했다는
근거가 이번 실험으로 확인됐다.

### 6단계 — 정리/저장
- `run_results/pure_glim_diag/`의 GLIM 내부 서브맵 바이너리 폴더(전체
  717개, dump_full_ct만 726M)는 재현 가능(같은 명령 재실행)하고 최종
  결과에 불필요해 삭제, 궤적 txt/graph.bin·txt/config/비교 PNG는 보존
  (12M로 축소).
- 프로세스: `pgrep -af "ros2|gz sim|glim"` → 잔여 없음(깨끗) 확인.
- 디스크: 72%(22G 여유) — 안전.
- 신규 config 디렉터리(`glim_config_cpu_std/`, `glim_config_ct_std/`)와
  신규 스크립트(`pure_glim_build_map.py`, `pure_glim_metrics.py`) +
  결과물(`pure_glim_result.md`, `pure_glim_map.png`, `pure_glim_map.npz`,
  `pure_glim_metrics.json`) 전부 git add 대상.
- 소요시간: 세션 시작~완료 약 35분(GLIM 로그 타임스탬프 18:03:35~18:30
  기준) — 3시간 타임아웃 안에 여유 있게 완료.

## [새 세션] GPS 결합 2종 비교(A: 사후결합, B: 실시간결합)

**목표**: 직전 세션이 확보한 "순수 GLIM" 궤적(GPS없음, wheel FN 33.39%,
최대오차 199.94m)에 A(사후결합, GLIM 재실행 불필요)와 B(실시간결합,
GLIM 전체 재실행 필요)를 각각 적용해 비교. 전체 타임아웃 3시간.

### -1단계 재개 확인
`pgrep -af "ros2|gz sim|docker"` → dockerd 외 없음(깨끗). `df -h ~` →
70%(23G 여유), 정리 불필요. branch/작업폴더 확인 완료.

### A단계 — GPS 사후결합

**A-1**: `glim_gps_correct.py`를 순수 GLIM 전체 궤적(`pure_glim_diag/
dump_full_ct/traj_lidar.txt`, 21255개 pose)에 적용. 자유낙하 구간
제외를 위해 t<=2129.67s로 먼저 트리밍(21207개 pose 남음, awk로 필터)한
뒤 입력. bag의 `/drone/gps`(22509개 샘플, t=[8.90,2259.70]) 재사용,
재비행 없음. **보정 전 GPS-GLIM 잔차: mean=242.175m, max=757.416m**
(원시 GLIM이 GPS/GT 절대좌표와 그만큼 떨어져 있었다는 뜻 — 로컬 프레임
오프셋 + 누적 드리프트가 합쳐진 값). 보정된 궤적 21207개 pose 저장.

**A-2**: `pure_glim_build_map.py`에 `align_window_s<=0`(신규 옵션,
GPS로 이미 world 프레임에 맞춰진 궤적을 다시 초기정합하면 이중보정이
되므로 건너뜀) 옵션을 추가해 지도 재생성. 결과:
- 격자 2481x2780(≈248x278m, 순수GLIM의 283x168m보다도 더 큼 — 아래 해석 참고)
- 커버리지 **99.98%**(999,845/1,000,000) — 순수GLIM(40.99%) 대비 대폭 개선.
- **wheel FN% 99.76%**(616,906/618,378), leg FN% 98.00% — 순수GLIM
  (33.39%/33.12%)보다 오히려 **크게 악화**.
- 최대연결덩어리% wheel/leg 둘 다 **0.00%**.
- 안전장치2(max_grid_cells) 발동 안 함(요청 6,897,180 < 상한 30,000,000).

**해석(중요, 예상 밖 결과지만 재현 가능한 실측)**: 지도의 실제 고도값
분포를 확인하니 min=-93.6m, max=101.1m, std=17.98m — 5m AGL 비행에서
나올 수 없는 극단적 노이즈다(버그 아님, 격자/정합 로직 재확인함).
`glim_gps_correct.py`는 GPS 앵커마다 **translation만** 보정하고
회전은 항상 identity 유지(설계 자체가 그럼, 3-3절 근거) — 즉 GPS는
"전체가 어디 있어야 하는지"(느리게 변하는 절대 위치 오프셋)는 강하게
고쳐주지만, CT 궤적 자체에 이미 있던 **국소적(스캔 간) 노이즈/자세
오차는 전혀 못 고친다**. 순수GLIM에서는 이 국소 노이즈가 있는 스캔들
중 다수가 애초에 평가영역 밖으로 새어나가 "미측정"(66%)으로 빠졌던
것인데, GPS로 전체를 평가영역 안으로 강제로 끌어오니 그 국소 노이즈가
전부 "관측됨"으로 드러나면서 인접 셀 높이차가 극단적으로 커져 거의
전부 "막힘"으로 잡힌 것으로 판단된다. **커버리지는 GPS 사후결합이
극적으로 개선하지만(위치는 맞게 되므로), FN%는 오히려 순수GLIM보다도
나빠질 수 있다** — "GPS 사후결합은 저주파(느린 드리프트) 오차만
고치고 고주파(국소) 노이즈는 못 고친다"는 게 이번 실측의 핵심 발견.

### B단계 — GPS 실시간결합

**B-1**: `glim_config_ct_std_gnss/` 신규 구성(순수 표준 CT + `libgnss_global.so`
만 추가, 다른 파라미터 튜닝 없음). glim_rosbag(배치)은 gnss_global처럼
라이브 ROS 토픽 구독이 필요한 확장 모듈과 아키텍처가 안 맞아(내부적으로
bag을 직접 읽어 콜백을 호출하는 방식이라 외부 프로세스가 그 "재생 진행"에
맞춰 GPS를 실시간으로 발행할 방법이 없음) 예전 방식(라이브 3-프로세스:
`ros2 bag play --clock` + `glim_rosnode` + `navsat_to_gnss_pose.py`)으로
복귀.

**B-2**: 자유낙하 구간 제외를 위해(이미 검증된 방법) `/drone/path_status`를
폴링하다 true 감지 즉시 bag play SIGINT → GLIM SIGINT(120초 대기 후
SIGTERM 승격) 순서로 종료하는 모니터를 세팅해 실행. 결과:
- **GPS 제약 활성화 확인**: `T_world_utm=se3(36.70,157.50,-8.34,...)`
  로그, 실행 35초 만에 1회(glim_ext 설계상 최초 1회만 정렬).
- **크래시 없음** — 예전 Phase 4 1차 시도의 "자유낙하 구간 세그폴트"가
  이번엔 재현 안 됨(이 SIGINT 절차가 실제로 막았다는 근거). 메모리도
  안정적(RSS 최종 수GB대, tick별 heartbeat로 실시간 관찰, OOM 없음).
- **그러나 라이브 파이프라인에서 심각한 프레임 손실**: "large time gap
  between consecutive LiDAR frames" 582회, "large time difference
  between points and imu" 9,593회(비행 내내 산발적) — 궤적 포즈 수
  9,565개로 배치 방식(21,255개)의 약 45%에 불과.

**B-3**: 자유낙하 이후분 트리밍(9526개 pose) 후 지도 생성
(`align_window_s<=0`, gnss_global이 이미 world 프레임으로 정렬했다고
간주). 결과: 커버리지 6.41%(미측정 91.0%), wheel FN% 8.69%(⚠️),
leg FN% 8.57%(⚠️), 최대연결덩어리% 0.00%/0.00%. 고도값 min=-84.4m
max=254.4m mean=127.8m std=62.9m — 궤적 전체가 순수GLIM/A보다도 훨씬
넓게(≈309x304m) 흩어짐. **wheel FN%가 낮아 보이는 건 미측정 91%짜리
극소 표본(운 좋게 GPS 정렬 직후의 안정적인 초반 구간)에서 나온 것이라
다른 조건과 액면가로 비교 불가** — 지시사항대로 억지로 다른 방식으로
재시도하지 않고 있는 그대로 기록.

**예전 실패 양상과 비교**: Phase 4 1차(세그폴트로 완전손실)/2차(GPS
미활성화) 둘 다와 다른, **제3의 실패 양상**(크래시도 없고 GPS도
걸렸지만 라이브 프레임 손실로 결과 신뢰 불가)이 나타남 — 매번 다른
방식으로 문제가 생긴다는 것 자체가 이 모듈/아키텍처의 근본적
비신뢰성을 오히려 더 뒷받침한다.

### 종합 비교 및 최종 결론

| 조건 | wheel FN% | leg FN% | 최대연결%(wheel/leg) | 커버리지 |
|---|---|---|---|---|
| baseline #5(칼만필터+GT) | 10.91% | 8.03% | 미산출 | 95.80% |
| 순수GLIM(GPS없음) | 33.39% | 33.12% | 0.00%/0.00% | 40.99% |
| A: GPS사후결합 | 99.76% | 98.00% | 0.00%/0.00% | 99.98% |
| B: GPS실시간결합 | 8.69%⚠️ | 8.57%⚠️ | 0.00%/0.00% | 6.41%⚠️ |

A는 커버리지를 크게 개선하지만 FN%는 순수GLIM보다도 악화(저주파 드리프트만
고치고 국소 노이즈는 그대로 노출). B는 표본이 너무 작아(6.41%) 비교
불가. **A도 B도 baseline #5를 넘어서지 못했고 둘 다 채택 비권고.**

**GLIM 관련 실험 전체(GICP, 완전독립추정, 파라미터튜닝, 실시간GPS×2,
사후결합GPS×1, 순수GLIM표준재현) 최종 결론**: GLIM 자체가 이 환경(GPU
없는 VM + 이 시뮬레이터 센서 특성 + 저고도 lawnmower 경로)에서 신뢰할
수 있는 궤적을 못 만든다는 게 근본 원인이고, GPS를 사후에 붙이든
실시간으로 붙이든 이 근본 문제는 해결되지 않았다. baseline #5(GT
pose 신뢰 + 셀단위 칼만필터+이상치방어)가 시도한 모든 대안 중 유일하게
안정적으로 좋은 결과를 내는 방법임이 최종 확정됐다 — 이 프로젝트의
커스텀 파이프라인은 결과를 포장한 게 아니라 실제로 필요했던 해법이었다.
상세 분석은 `run_results/pure_glim_gps_both_result.md` 참고.

### 정리/저장
- `pure_glim_diag/dump_gnss_live/`의 서브맵 바이너리(717개 상당, 465M)는
  삭제(재현 가능, 최종결과 불필요), 궤적/graph/config/PNG는 보존(4.1M로 축소).
- 잔여 프로세스 없음 확인(ros2-daemon만 정상 상주). 디스크 71%(22G 여유).
- 소요시간: 세션 시작~완료 약 1시간(로그 타임스탬프 21:11~21:49 기준
  B 실행이 대부분, A는 수분 내 완료) — 3시간 타임아웃 안에 여유 있게 완료.

## [새 세션] Module A 칼만필터 11가지 개선 후보 실험 (2026-08-20)

**목표**: baseline #5(칼만필터+GT pose, wheel FN% 10.91%, leg FN% 8.03%,
커버리지 95.80%) 코드에서 "정확히 하나만" 바꾼 11가지 변형을 각각
baseline과 1:1 비교(같은 bag, 같은 파이프라인, 변경점 1개만), 유효한
것들을 조합. 전체 타임아웃 8시간.

### -1단계 재개 확인 + 디스크 정리
`pgrep -af "ros2|gz sim|docker"` → dockerd만(깨끗). `df -h ~` → 71%(22G
여유). 사용자가 세션 중간에 "저장공간 부족하면 이전 실험 불필요 파일
정리하며 진행"이라고 추가 지시 — 이미 결론이 끝난 GLIM 실험의 미커밋
로컬 전용 대용량 파일(`pure_glim_gps_postfusion_map.npz` 106M,
`pure_glim_gps_realtime_map.npz` 144M, `pure_glim_map.npz` 73M, 합계
323M — 전부 스크립트+커밋된 궤적으로 재생성 가능, 보고서/결론은 이미
커밋됨)을 즉시 삭제. baseline #5 bag은 이미 삭제된 상태(`bags/
velocity_4m_5mps_attempt1` 없음) — GPS 포함 bag(`bags/
velocity_4m_5mps_gps_attempt2`, 23G)이 동일 조건(5m AGL/4m/5mps
velocity)이라 이걸로 대체 재사용(재비행 없음, GPS 토픽은 이 실험과
무관해 존재 여부가 결과에 영향 없음).

### 0단계 — 재생 속도(rate) 검증
매 실험이 bag 전체(2254s)를 rate=1.0(실시간)으로 재생하면 회당 ~37분,
11개 변형만 해도 6.8시간+로 8시간 예산을 거의 다 쓴다. drone_elevation_
mapper는 GLIM처럼 무거운 정합이 없는 벡터화 칼만필터라 더 빠른 처리가
가능할 것으로 판단, **rate=2.0으로 baseline(무수정 코드)을 먼저
재현**해 결과가 기존 baseline #5(10.91%/8.03%/95.80%)와 충분히
가깝게 나오는지 검증한다 — 가까우면 이후 모든 실험에 rate=2.0을
채택(예산을 절반으로 줄임), 크게 벗어나면 rate=1.0으로 전부 되돌린다.
결과는 아래에 기록.

**결과**: rate=2.0 재현 — wheel FN% 10.68%(baseline #5: 10.91%, 차이
0.23pp), leg FN% 7.89%(8.03%, 차이 0.14pp), 커버리지 95.89%(95.80%,
차이 0.09pp) — 전부 오차범위 내로 매우 근접. **rate=2.0을 이번 세션
전체(모든 Phase 1/2 실험)의 표준으로 채택.** 이 rate=2.0 결과
(`run_results/kf_variants/baseline_check_fn.json`)를 이후 모든 변형의
1:1 비교 기준(local baseline, 같은 bag/rate/파이프라인이라 가장
엄밀함)으로 쓴다. 소요시간 확인: 약 11분(2254s/2≈1127s 재생 + 오버헤드) —
예상대로 rate=1.0(37분) 대비 대폭 단축, 11개 변형+조합+연결성 검증까지
8시간 예산 안에 여유 있게 가능하다고 판단.

### Phase 2 — 조합 실험

**조합 구성**: 11(건물경계 혼합모델, 게이팅 전략으로 채택)의 내부 임계값을
16.0으로(게이트16 반영) + 8(R 가중합) + 9(방향성 R) + 10(적응형 R). 게이트16/
G/11은 같은 코드 지점("이노베이션이 임계값을 넘으면 무엇을 할지")을 서로
다른 방식으로 바꿔 상호배타적이라, 셋 중 최고 성적인 11을 게이팅 전략으로
채택하고 그 안의 임계값만 16으로 올렸다(파라미터라 독립 조합 가능). 8/9/10은
서로 다른 코드 위치(R 결합식/방향성 배율/전역 스케일)를 건드려 전부 함께
조합 가능해 모두 포함.

`drone_elevation_mapper.py`에 파라미터 7개(building_mixture_enabled,
r_combination_mode, incidence_sigma_base_m, directional_r_enabled,
along_track_extra, adaptive_r_enabled, adaptive_r_alpha) + 상태
4개(elevation_b/variance_b, prev_sensor_xy/heading, adaptive_r_scale) 추가,
`_grow_to_fit`(B그리드 동반 패딩)/`_accumulate_common`(heading 추정+분기)/
`_measurement_noise`(8+9 결합)/신규 `_kalman_update_cells_mixture`(11+10
결합)/`_build_grid_map_message`(승자가설 선택) 수정. 크래시 없이 완주.

**결과**: wheel FN% 4.35%(11단독 4.38% 대비 -0.03pp), leg FN% 4.29%(11단독
4.28% 대비 +0.01pp), 커버리지 95.89%(11단독 95.98% 대비 -0.09pp) — **11
단독과 사실상 통계적으로 구분 불가.** 최대연결덩어리%도 wheel 51.97%/leg
54.52%로 11단독(51.63%/54.51%)과 거의 동일.

**핵심 발견**: 5개를 다 합쳐도 11 단독보다 나아지지 않는다 — 개선분이
"상쇄"는 아니지만 "중복(redundant)"이다. 8/9/10(R모델 개선 3종)은 전부
"baseline 고정 R이 관측을 과신하게 만들어 좋은 관측까지 하드 게이트가
거부한다"는 같은 근본 원인을 다른 각도에서 완화하려던 시도였는데, 11은
그 근본 원인의 결과(거부당한 관측) 자체를 안 버리고 두 번째 가설로
보존해버려서 R을 아무리 정교하게 재추정해도 게이트 거부 자체의 영향력이
11 아래에서는 이미 크게 줄어든 상태였던 것으로 해석된다.

### 연결성 최종 검증 (지시된 코드 그대로, `kf_connectivity_check.py`)
| | baseline | 11(혼합모델) | 조합 |
|---|---|---|---|
| wheel 최대연결% | 27.58% | **51.63%** | 51.97% |
| leg 최대연결% | 40.96% | **54.51%** | 54.52% |
"FN%만 좋아 보이고 실제로는 못 쓰는 지도"가 아님을 확인 — 11/조합 둘 다
연결된 통과가능 영역 자체가 baseline보다 훨씬 넓다.

### 최종 추천
**단독 최우수·실무 권고: 11(건물경계 혼합모델) 단독 채택.** 8/9/10/게이트16을
추가로 얹을 이유가 없다(중복, 구현 복잡도만 증가). 11의 구현 복잡도가
부담되면 차선책은 G(Huber, 구현이 단일 블록 교체로 훨씬 단순, FN%가 11과
0.24~0.34pp밖에 안 차이남).

### 코드 최종 상태 확인
전체 11개 Phase 1 + Phase 2 조합 실험 종료 후 `git diff --stat
src/agconav_drone/agconav_drone/drone_elevation_mapper.py` → **0줄(완전히
baseline #5 상태)** 확인 완료. 어떤 실험적 변경도 코드에 남기지 않음 —
전부 문서(`kf_experiment_summary.md`, 개별 `kf_variant_*.md`)로만 보고.

### 정리/저장
- 잔여 프로세스 1개(combo 실행분 drone_elevation_mapper) 발견해 정리
  완료, 이후 전부 깨끗함 확인.
- 디스크 71%(22G 여유) — 안전, 추가 정리 불필요했음.
- `run_results/kf_variants/maps/`(baseline·11·combo_p2 elevation_map
  mcap, 각 8MB)는 `.gitignore`(`maps/`)에 걸려 로컬에만 보존 — 재현
  명령은 각 문서에 정확히 기록됨.
- 소요시간: 세션 시작~완료 약 4시간 10분(로그 타임스탬프 00:53~04:58
  기준 실험 실행 구간, 이후 문서화·정리 포함) — 8시간 타임아웃 안에
  여유 있게 완료. 11개 Phase 1 항목 전부 시도(스킵 없음), Phase 2
  조합까지 완료.

## [새 세션] FAST-LIO2 실험 (2026-08-20)

**목표**: FAST-LIO2(GLIM과 무관한 독립 패키지)를 5m AGL/4m간격/5m/s
조건에서 테스트, GPS 없이 단순평균(칼만필터 없음) 지도로 순수GLIM
(wheel FN 33.39%)과 사과 대 사과로 비교. 전체 타임아웃 4시간.

### -1단계 재개 확인
`pgrep -af "ros2|gz sim|docker"` → dockerd만(깨끗). `df -h ~` → 72%(22G
여유), 정리 불필요. branch/작업폴더 확인 완료.

### 0단계 — 빌드
`~/fastlio_ws/src`(이 저장소와 완전히 별도)에 `MIT-SPARK/spark-fast-lio`
clone(README에 명시적으로 "ROS2 Jazzy" 배지 — 우리 환경과 정확히
일치, 좋은 신호). `git submodule update --init --recursive`로
`ikd-Tree` 서브모듈 확보. `package.xml`/`CMakeLists.txt` 의존성 확인
(rclcpp/tf2_*/pcl_ros/pcl_conversions/Eigen3/PCL, livox_ros_driver는
QUIET로 옵션) — 전부 표준 ROS2 패키지라 우리 환경(GLIM 빌드 때 이미
tf2_sensor_msgs 등 확보)에 이미 있을 가능성이 높다고 판단, 별도
사전조치 없이 바로 `colcon build --packages-up-to spark_fast_lio`
시도. 결과는 아래에 기록.

### 0단계 결과 — 빌드 성공
`colcon build --packages-up-to spark_fast_lio --symlink-install` 성공
(~3.5분, IKFoM/ESKF 템플릿 코드라 느릴 뿐 정상). 실행파일
`spark_lio_mapping` 확보. 이 VM은 aarch64(ARM64) 아키텍처였음(빌드 로그로
처음 확인 — 지금까지 별문제 없었음).

### 0-2단계 — 설정 구성 (`~/fastlio_ws/agconav_config.yaml`, 이 저장소
바깥, 지시사항대로 test_main_brian 소스에는 아무것도 안 넣음)
- `lidar_type=3`(OUST64), `scan_line=32`(OS1-32 실제 채널 수),
  `timestamp_unit=3`(나노초).
- `blind=2.5`, `det_range=200.0` — 각각 이 프로젝트 전체에서 일관되게
  써온 `min_range_m`/`max_sensor_range`와 동일 값(정확도 튜닝이 아니라
  자기반사 제거·사거리 상한 방어라는 동일한 목적의 파라미터를 그대로
  이식).
- `extrinsic_T=[0,0,-0.135406]`, `extrinsic_R=Ry(+90°)`(model.sdf 직접
  실측: os1_lidar pose=(0,0,-0.175406,pitch=+90°), IMU pose=(0,0,-0.04,
  무회전), 둘 다 model 프레임 기준. FAST-LIO 컨벤션(T_imu_lidar, "LiDAR
  w.r.t. IMU")에 맞게 직접 계산). GLIM 실험 때 쓴 T_lidar_imu(반대 방향)
  의 역행렬과 정확히 일치함을 대수적으로 교차검증. 이후 실제 bag 점을
  이 extrinsic으로 변환해 IMU 프레임 Z가 -5.36~0.26m(평균 -4.4m, 5m AGL
  비행에 부합)로 나오는 것으로 추가 실측 검증까지 완료(아래 버그 수정
  전 진단 과정에서 확인).
- `common.visualization_frame: "lidar"` — odometry 토픽이 T_map_lidar를
  직접 발행하게 해서 우리 맵빌더가 GLIM 실험 때처럼 IMU-LiDAR 합성을
  또 할 필요가 없게 함(설정 선택일 뿐 정확도 튜닝 아님).
- `gravity_alignment.enable_gravity_alignment`: 패키지 기본값(true)
  유지(우리 bag이 실제 정지 구간에서 시작하므로 조건 부합) — 나중에
  off로도 대조 실험(아래).
- 나머지(acc_cov/gyr_cov 등 IMU 노이즈 사전값, point_filter_num,
  filter_size_map 등)는 패키지 제공 템플릿(`config/ouster_vbr.yaml`)
  기본값 그대로, 우리 데이터에 맞춘 재추정 없음.

### 0-3단계 — per-point 타임스탬프 이슈: 필수로 확인, 브리지 신규 작성
`oust64_handler`(spark_fast_lio 소스 직접 확인) 코드가 `pl_orig.points[i].t`
를 **무조건 직접 읽어 큐레이처(모션보정용 스캔내 상대시각)로 사용** —
GLIM처럼 없으면 자동으로 점순서 근사로 대체하는 옵션이 없다. 필드
누락 시 동작이 정의돼 있지 않음(사실상 필수) → GLIM 디스큐 실험과
동일한 방위각(column index) 기반 근사로 t필드를 신규 계산해 추가하는
브리지 노드(`run_results/fastlio_points_republisher.py`, 신규)를 작성.
`ouster_ros::Point` 9개 필드(x,y,z,intensity,t,reflectivity,ring,ambient,
range) 레이아웃에 맞춰 재발행 — PCL `fromROSMsg`는 메시지 자체의
`fields[].offset`로 이름 매칭하므로 C++ 구조체의 실제 메모리 정렬을
복제할 필요 없음(직접 패킹해도 무방, 확인 후 결정).

부수 발견: FAST-LIO의 lidar 구독 QoS가 **reliable**(코드 실측 확인,
`/opt/ros/jazzy`가 아니라 spark_fast_lio.cpp 자체)인데 원본
`/drone/points`는 이 프로젝트 전체가 그렇듯 best_effort로 발행돼
그대로 remap하면 아예 연결이 안 된다 — 브리지가 QoS도 reliable/
volatile로 바꿔 발행해 이 문제도 함께 해결(브리지 노드 자체 목적과
자연스럽게 겹치는 부수 효과, 별도 우회 아님). imu는 둘 다
best_effort라 그대로 remap.

궤적 기록: FAST-LIO2는 GLIM과 달리 자체적으로 파일 저장을 안 함
(`save_dir_`/`sequence_name_` 파라미터는 선언만 되고 실제 파일쓰기
코드 없음, 소스 직접 확인). 대신 매 프레임 `odometry`(nav_msgs/Odometry)
토픽을 발행하므로 `run_results/fastlio_traj_recorder.py`(신규)로 받아
TUM 포맷 저장.

### 1단계 — 짧은 구간 테스트: 버그 2개 발견·수정, 최종 판정은 "발산"

**시도1(t=33~90s, GLIM 실험과 동일한 "첫 코너 전 깨끗한 구간")**: 즉시
`AssertionError: All fields need to have the same datatype`로 브리지가
죽음 — `read_points_numpy`는 단일 dtype만 반환하는데 x/y/z/intensity
(float32)와 ring(다른 타입)을 섞어 요청한 게 원인. `read_points()`
(구조화 배열, 이종 타입 지원)로 교체해 수정.

**시도2(수정 후, 같은 구간)**: 크래시는 없었지만 "No Effective Points!"
경고가 시작부터 끝까지 반복, 궤적이 t=98.48s에 (1054,1186,-2595)로
**파국적 발산**(수천 미터). 원인 조사: bag의 raw `/drone/points`가
gz gpu_lidar 특성상 무반사 점을 NaN이 아니라 **Inf로 채운다는 사실**
(이 프로젝트 전체에서 반복 확인된 사실, drone_elevation_mapper.py도
명시적으로 필터링)을 **브리지에서 빠뜨렸음을 발견** — Inf가 섞인
스캔이 FAST-LIO의 ikd-tree/ESKF 처리를 오염시킨 것으로 판단, `np.isfinite`
필터를 브리지에 추가해 수정.

**시도3(Inf 필터 수정 후, 같은 구간 t=33~90s)**: "No Effective Points"
경고 완전히 사라짐(파국적 발산의 직접 원인은 해결). 그러나 궤적은
여전히 **Z가 t=42.9~96s 동안 0.6m→68.9m로 거의 선형으로("등속") 표류**
(기울기 ≈1.29m/s) — 파국적은 아니지만 명백한 발산.

**추가 진단(설정 문제인지 재확인)**:
1. t=33 시작은 이미 순항비행 중이라 gravity_alignment의 "정지 구간
   필요" 전제가 깨졌을 가능성 → **t=0(실제 호버링 구간)부터 재시도**:
   같은 선형 Z 표류 패턴 재현(t=97에 Z≈67m) — **가설 기각**.
2. gravity_alignment을 아예 꺼도(`enable_gravity_alignment:=false`,
   ouster_vbr.yaml 예제가 쓰는 방식) 결과 사실상 동일(t=98.4에 Z≈69.5m)
   — **역시 원인 아님**.
3. GT 대조(`compare_gt_glim.py` 재사용, traj_lidar.txt를 odom_imu.txt로
   복사해 그대로 씀): **발산 자동탐지 t=10.28s**(사실상 궤적 시작
   직후부터), 최대오차 132.85m(88초 구간, t=98.48s) — GLIM의 순수 실험
   (같은 bag, 최대오차 199.94m/2129초)과 자릿수가 비슷한 발산 양상.

**판정**: extrinsic은 실측 데이터로 직접 검증 완료(IMU프레임 변환 후
Z가 -5.36~0.26m, 5m AGL과 부합), Inf 필터·시작시점·중력정렬까지
전부 확인했는데도 같은 패턴이 재현되므로, **이건 설정 버그가 아니라
이 bag/센서 데이터 자체의 근본적 어려움(GLIM 실험에서 이미 확인된
"반사가 방위 180~270° 한 구획에만 몰리는" 희소한 반사 패턴 + 84m
아님 5m 저고도지만 여전히 얕은 입사각 다수)이 FAST-LIO2에도 동일하게
악영향을 준 것으로 판단**. 지시사항대로("문제가 나타나면 정확히
기록하고 우회하지 마라") 더 이상 새 우회를 시도하지 않고, 크래시가
아닌 "발산"임을 기록한 뒤 2단계(전체 bag 확장, 크래시 여부만 정지
기준)로 진행한다.

### 2단계 — 전체 bag 확장: 완료 (크래시 없음), 세션 일시중단

**전체 bag(t=0~2129.67s, 자유낙하 구간 제외) 실시간(rate=1.0) 처리
완료** — bag 재생 자체는 16:21:12 시작~16:57:00 종료(약 35.8분,
예상 35.5분과 거의 일치). **크래시/OOM 없음**, 경고("No Effective
Points"/"No point, skip this scan")도 전체 구간에서 단 18건뿐(대부분
맨 끝 경계 근처로 추정) — 1단계에서 봤던 것과 달리 이번엔 거의 깨끗하게
완주했다. 궤적 220,120개 포즈, t=[?,2135.34]까지 기록(요청한
playback-duration=2129.67보다 살짝 넘어간 것은 GLIM 때와 동일한 흔한
버퍼링 오버런 — t<=2129.67로 트리밍해 자유낙하 제외 정책 그대로 적용).

**세션 중단 사유**: 사용자가 귀가해야 해서 "지금 실행 끝나면 잠깐
멈춰달라"고 요청 — 3단계(지도 생성)·4단계(지표 산출)·5단계(비교)는
**의도적으로 미완료 상태로 남김**(다음 세션에서 이어받을 것).

**알려진 사소한 이슈(다음 세션이 알아둘 것)**: `run_fastlio.sh`의 종료
시퀀스(`kill -INT` → sleep 3 → 나머지 kill → sleep 1 → `kill -9` 전체)가
`spark_lio_mapping`을 못 죽이고 프로세스가 남는 현상 실측됨(bag play/
republisher/recorder는 정상 종료됐는데 fastlio 노드만 남음, 이유
미상 -- `set -e`와 `kill` 여러 PID 중 일부가 이미 죽은 상태에서의 종료
코드 상호작용으로 추정, 확정 원인 조사는 안 함). 데이터 자체는 완전한
상태로 저장됐으므로 결과에 영향 없음. **다음 세션 시작 시 반드시
`pgrep -af "spark_lio_mapping"`로 잔여 프로세스 확인 후 정리할 것**
(이번엔 수동으로 `kill -9`해서 정리 완료해뒀음).

### 저장된 산출물 (재개용)
- `run_results/fastlio_diag/traj_lidar_full_raw.txt`(220,120개 포즈,
  원본 그대로) / `traj_lidar_full_trimmed.txt`(219,537개 포즈, t<=2129.67
  트리밍 완료, **다음 세션은 이 파일을 바로 3단계 지도 생성에 쓰면 됨**).
- `run_results/fastlio_diag/full_fastlio.log`, `full_play.log`(전체
  실행 로그, 경고 18건 포함).
- 앞서 커밋된 `run_results/fastlio_build_map.py`(3단계 지도 생성
  스크립트, 이미 작성+짧은 구간으로 스모크테스트 완료됨 -- 그대로
  실행 가능)와 `~/fastlio_ws/agconav_config.yaml`(설정, 이 저장소
  바깥이라 git 추적 대상 아님, 별도 백업 불필요할 만큼 이미
  PROGRESS.md에 전체 내용 기록됨).

### 다음 세션이 이어받을 지점 (정확한 다음 명령)
```
cd ~/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 run_results/fastlio_build_map.py \
  bags/velocity_4m_5mps_gps_attempt2 \
  run_results/fastlio_diag/traj_lidar_full_trimmed.txt \
  run_results/fastlio_map.npz \
  -inf 2129.67 30000000 5.0
```
이후: `run_results/pure_glim_metrics.py`와 동일한 패턴(또는 그 스크립트를
그대로 재사용 -- 입력 npz 포맷이 동일하게 설계됨, `elevation`/`origin_x`/
`origin_y`/`res` 키 동일)으로 4가지 지표(커버리지/wheel·leg FN%/
wheel·leg 최대연결덩어리%) 계산, 비행 소요시간은 이미 확정 가능(bag
`/drone/path_status` 기준 2129.674초, GLIM 실험과 동일한 bag이므로
동일값). 이후 `run_results/fastlio2_result.md` 작성, SUMMARY.md/
PROGRESS.md 갱신, git add/commit/push, 대화창에 종합비교표 출력까지가
남은 작업.

세션 소요시간: 시작~중단 약 1시간 15분(4시간 예산 중 여유 많이 남음) --
다음 세션에서 남은 작업(지도생성~보고)에 필요한 시간은 20~30분 정도로
추정(전부 스크립트는 이미 작성/검증 완료 상태).

### [재개 세션] 3~6단계 완료

재개 시 `pgrep -af "ros2|gz sim|docker|spark_lio_mapping|fastlio"` →
잔여 없음(깨끗, 이전 세션에서 이미 정리됨) 확인.

**3단계 — 지도 생성**: `fastlio_build_map.py`로
`fastlio_diag/traj_lidar_full_trimmed.txt`(219,537개 포즈) 처리.
격자 1303x1287(≈130x129m), 유효 셀 886,549개(52.87%). **안전장치
(max_grid_cells=30,000,000) 발동 없음**(요청 최대 1,676,961셀).
지도 시각화(`fastlio_map.png`)에서 반복되는 "파이프/원통" 패턴 확인 —
자세(회전) 오차 누적으로 같은 지형이 여러 각도로 겹쳐 찍힌 것으로 해석.

**4단계 — 4가지 지표**: 비행시간 2129.674초(35.49분, 동일 bag이라
GLIM 실험과 같은 값). 커버리지 30.63%(부분 커버리지, 비행 초반에
편중된 조각). wheel FN% 2.97%/leg FN% 2.28%(미측정 71.2% — **낮은
커버리지 때문에 액면가 비교 금지**, 보고서 4-3절에 명확히 경고 기록).
최대연결덩어리%(지시된 코드) wheel 13.54%/leg 15.09% — 순수GLIM(0%/0%)
보다 뚜렷이 개선, baseline #5(27.58%/40.96%)·11번(51.63%/54.51%)에는
못 미침.

**5단계 — 종합 비교**: FAST-LIO2가 순수GLIM보다 최대연결덩어리%
(체계적/선형 드리프트 덕)와 크래시 내성 면에서 낫지만, 커버리지가
baseline #5에 크게 못 미쳐(30.63% vs 95.80%) 실무 대안은 아님. FN%가
낮아 보이는 이유(비행 초반 편중 표본)를 명확히 밝힘.

**6단계 — GPS 결합 검토**: GLIM 때(국소 노이즈라 GPS로 해결 안 됨)와
달리 FAST-LIO2의 드리프트는 거의 선형/체계적이라 GPS 사후결합이 잘
들을 유형으로 판단, **시도해볼 가치 있음(GLIM보다 우선순위 높게)**
으로 결론(단, 커버리지 자체의 근본 한계는 GPS로 해결 안 됨을 명시,
제한된 목표로 접근 권고).

### 정리/저장
- 잔여 프로세스 없음 확인. 디스크 72%(21G 여유) — 안전, 추가 정리
  불필요.
- `run_results/fastlio2_result.md`, 갱신된 `SUMMARY.md`/`PROGRESS.md`,
  `fastlio_map.png`/`.npz`, `fastlio_metrics.json` 전부 커밋 대상.
- 세션(중단 전+재개 후) 총 소요시간: 약 1시간 40분(4시간 예산 중
  여유 많이 남음) — 타임아웃 걱정 없이 전 단계(0~6) 완료.

### 커버리지 30.63% 원인 추가 분석 (사용자 질문에 답하며 실측)

정합된 궤적으로 직접 확인: 전체 219,537개 포즈 중 38.2%(83,871개)만
평가영역(100x100 박스) 안에 있었고, **t=1329.38초(비행의 약 62%
지점)를 마지막으로 궤적이 박스를 완전히 벗어난 뒤 남은 약 800초
(38%) 동안 한 번도 돌아오지 않았다**(그 전까지는 박스 안팎을 12번
들락날락). 즉 드론은 실제로 100x100 전역을 계속 비행했지만, 초반
15초 GT 정합 이후 보정이 전혀 없어(GPS/후처리 없음) 누적 표류가
비행 후반부(약 62% 지점)에 박스 크기(100m)를 초과하면서 이후
관측점들이 전부 "박스 밖" 좌표로 잘못 찍혔다 — 이게 커버리지
30.63%(38.2%보다도 낮은 건 겹치는 스캔들이 새 셀을 못 채워서)의
직접 원인. GLIM(평균 68.6m 오차, 왕복마다 진동)과 달리 FAST-LIO2는
표류가 단조증가형이라 "벗어난 뒤 다시 안 돌아옴" 패턴이 이번에
고유하게 나타남 — 근본 원인(희소·편중 반사 패턴)은 동일.

## [사용자 요청] FAST-LIO2 + GPS 사후결합 시도

사용자가 GPS 결합 재시도를 요청, spark-fast-lio 소스 전체 검색
(`grep -rli "gps|gnss|navsat"`) 결과 **GPS/GNSS 관련 코드 전혀 없음**
확인 — GLIM(glim_ext)과 달리 실시간결합은 커스텀 C++ 개발이 필요함을
사용자에게 안내, 사용자가 **A(사후결합)만 먼저 진행**하기로 선택.

`glim_gps_correct.py`(범용 TUM 스크립트, GLIM 전용 아님)를 FAST-LIO2
저장 궤적에 그대로 재적용(재실행 없음, 몇 분 완료). 결과: 커버리지
30.63%→98.75%, **wheel FN% 2.97%→97.50%로 폭등, 최대연결덩어리%
0%로 붕괴** — GLIM 사후결합 실험과 정확히 같은 실패 패턴. "GPS
사후결합이 FAST-LIO2엔 더 잘 들을 것"이라던 6절의 가설은 **틀린
것으로 확인**, `fastlio2_result.md` 6절과 `SUMMARY.md`에 정정 반영.
상세: `run_results/fastlio_gps_postfusion_result.md`(신규).

**최종 결론(GPS 결합 전체)**: GLIM·FAST-LIO2 둘 다 GPS 사후결합은
채택 비권고 — 도구나 드리프트 형태와 무관하게 "저주파만 고치고
국소노이즈는 못 고친다"는 사후결합 방식 자체의 구조적 한계로 확정.
실시간결합(B)은 사용자가 이번엔 보류(추후 필요시 ESKF 커스텀 개발).
