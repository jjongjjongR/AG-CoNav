# agconav_map_fusion  (담당: 채현우)  — 모듈 E (메인 결과물)

드론·wheel·leg의 2.5D 지도를 공통 map 좌표에서 하나로 병합(fusion)하는 패키지.

- 입력: /drone/elevation_map, /wheel/elevation_map, /leg/elevation_map.
- 출력: 통합 2.5D 지도(/merged_map), 저장.
- multirobot_map_merge가 Jazzy 미지원 -> 커스텀 노드로 처리.
- 미결정(TBD): 중복 셀·미관측 셀·서로 다른 범위/해상도 처리 규칙.
