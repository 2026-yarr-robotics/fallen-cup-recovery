# outlier-cup-recovery — 안전 궤적·안전 제어 로직 정리

`./start.sh --real-api` 로 도는 컵쌓기 파이프라인에서, 외란(쌓다 넘어진 컵 ·
입구가 위를 향한 컵)을 치우는 **outlier-cup-recovery** 모듈이 MoveItPy 로
"안전한 궤적을 만들고 안전하게 제어"하기 위해 적용한 기법들을 한 곳에 모았다.

대상 코드:

```
outlier-cup-recovery/dsr_practice/dsr_practice/
├── outlier_cup_recovery.py   # 통합 오케스트레이터 (fallen → mouth-up 라우팅)
├── stand_fallen_cup.py       # 넘어진 컵 세우기 (대부분의 안전 로직이 여기)
├── place_mouth_up_cup.py     # 입구-위 컵 뒤집어 엎기
├── common.py                 # 공용 유틸 (작업영역 클램프, plan_and_execute)
└── config/moveit_py.yaml     # 플래닝 파이프라인 설정
```

> **요약** — 이 모듈의 안전 설계는 "한 가지 안전장치"가 아니라 **여러 겹의
> 독립적 방어선**이다. 위치 단계에서 막고(작업영역 클램프·하드 플로어), 충돌
> 단계에서 막고(그리퍼/피라미드/정상컵 collision object), 자세 단계에서 막고
> (IK seed 잠금·분기 가드), 플래너 단계에서 막고(PTP/LIN/OMPL 역할 분담 +
> 보수적 속도), 실행 단계에서 막는다(수직 상승·pre-reverse·도달성 사전검사).
> 한 겹이 뚫려도 다음 겹이 잡는 구조다.

---

## 0. 큰 그림: 왜 이렇게까지 하나

로봇은 **Doosan M0609 6축 + RG2 그리퍼**이고, 작업영역에는 이미 쌓은
**피라미드**와 **정상 컵**들이 놓여 있다. recovery 동작이 잘못된 궤적을 만들면

- 테이블을 그리퍼로 긁거나(수평 sweep),
- 쌓아둔 피라미드를 쳐서 무너뜨리거나,
- 손목 특이점을 가로지르며 관절 속도한계를 위반해 드라이버가 죽거나
  (`-6`/SIGABRT),
- IK 가 "천장으로 팔을 쳐들었다 내려오는" 다른 분기 해로 수렴해 크게 휘두르는

사고가 난다. 아래 기법들은 전부 **실로봇에서 한 번씩 터졌던 사고**를 막기 위해
하나씩 추가된 방어선이다(코드 주석에 사고 사례가 그대로 남아 있다).

---

## 1. 작업영역 안전 클램프 + 바닥 하드 플로어 (위치 레벨 방어)

### 1-1. 안전 박스 클램프 — `common.py:53` `clamp_to_safe_workspace`
모든 pose 목표는 플래닝 **전에** base_link 기준 안전 박스 안으로 강제 클램프된다.

```python
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.30 ; SAFE_Y_MAX = 0.30
SAFE_Z_MIN = 0.05     # link_6 flange 최저 안전 z
```

`plan_and_execute(..., clamp=True)`(기본값)가 목표를 받자마자 이 범위로 잘라낸
뒤에야 `arm.plan()` 을 호출한다(`common.py:69`). 즉 인식 오차나 좌표 계산 실수로
목표가 작업영역을 벗어나도, 플래너에는 **항상 안전 범위 안의 목표만** 들어간다.

### 1-2. 바닥 충돌 방지 하드 플로어 — `stand_fallen_cup.py:1758`
깊이(depth) 센서가 컵 표면을 실제보다 깊게 읽거나 z 가 음수로 튀어도, 하강
목표 z 를 `descend_min_z` 아래로는 **절대** 내리지 않는다.

```python
floor_z = max(SAFE_Z_MIN, self.descend_min_z)
if descend_z < floor_z:
    descend_z = floor_z   # 바닥을 박지 않도록 클램프
```

`descend_min_z` 기본값은 "컵이 테이블에 있을 때의 grasp flange z"로 자동
설정되어(`stand_fallen_cup.py:478`), 정상 테이블 grasp 깊이보다 깊게는 못 간다.
이 플로어는 실제 실행뿐 아니라 **사전 도달성 검사(preflight)에도 동일하게**
적용된다(§8).

---

## 2. 충돌 회피 — Planning Scene 에 장애물 3종 등록 (충돌 레벨 방어)

MoveItPy 의 planning scene 에 장애물을 등록해, OMPL 플래너가 **충돌을 인지하고
피해 가는 궤적**을 만들게 한다. 세 종류를 모두 등록한다.

### 2-1. 그리퍼 부피 프록시 (AttachedCollisionObject, BOX) — `stand_fallen_cup.py:2276`
RG2 그리퍼의 외형 부피를 `link_6` 에 박스로 **붙여서**(attach), 이후 모든 plan
이 "팔 링크"뿐 아니라 "그리퍼가 차지하는 부피"까지 장애물과의 충돌을 피한다.

```python
GRIPPER_PROXY_SIZE  = (0.16, 0.10, 0.20)   # (x,y,z) m
GRIPPER_PROXY_OFFZ  = 0.10                  # 플랜지 +Z 로 박스 중심 이동
GRIPPER_PROXY_TOUCH = ["link_6","link_5","link_4"]  # self-collision 무시
```

- 그리퍼는 sim/실로봇 양쪽에 물리적으로 항상 달려 있으므로 **기본 ON**.
- 끄면 플래닝이 그리퍼 부피를 무시해 피라미드를 클립(스칠) 위험이 생긴다.
- 노드 자체 MoveItPy scene(실제 plan 이 보는 scene)과 move_group scene(RViz
  시각화)에 **양쪽 다** 적용한다. 실로봇에 move_group 이 없으면 graceful skip
  (`stand_fallen_cup.py:2318` `_apply_aco_to_move_group`).

### 2-2. 쌓인 피라미드 장애물 (CollisionObject, BOX) — `stand_fallen_cup.py:2340`
이미 쌓아 둔 피라미드를 collision box 로 등록해 recovery 궤적이 피하게 한다.

- 점유 슬롯은 `/stack` 토픽에서, 중심/회전(center·degree)은 서버
  `GET /api/robot/config/pyramid` API 폴링에서 가져와 **vision verifier·FastAPI
  배치 기하를 그대로 미러링**한다(`PYRAMID_CUP_SPACING=0.078`,
  `PYRAMID_LAYER_PITCH=0.093`, 레이아웃 `[3,2,1]`).
- `pyramid_obstacle_margin_m`(기본 0.02m)만큼 박스를 인플레이션해 안전 여유 확보.
- z 는 테이블(`TABLE_Z`)에 앵커해 바닥부터 보수적으로 세운다.
- **서버·vision 이 없으면 graceful no-op**(장애물 0개, 동작 변화 없음) — recovery
  자체는 계속 진행된다.

### 2-3. 정상(세워진) 컵 장애물 (CollisionObject, CYLINDER) — `stand_fallen_cup.py:2418`
`/hand_eye/boxes` 가 보고하는 정상 컵 위치마다 실린더 장애물을 등록한다.

- multi-cup loop 는 **매 sense 직후** 이전 등록을 REMOVE 하고 최신 스냅샷으로
  다시 ADD 한다(`stand_fallen_cup.py:1560`). 컵이 옮겨지거나 사라져도 scene 이
  실제와 정합을 유지한다.
- 반경/높이: `upright_obstacle_radius_m`(0.04) · `upright_obstacle_height_m`(0.12).

---

## 3. IK seed 잠금 + 분기(branch) 가드 (자세 레벨 방어)

6축 로봇의 IK 는 한 목표 pose 에 대해 **여러 해(분기)**가 있다. 수치 IK 솔버가
멀리 튀어 "손목 뒤집힘(wrist-flip)"이나 "팔꿈치 위로(천장) 분기" 해로 수렴하면,
거기까지 가려고 팔을 **크게 휘두르는 위험 궤적**이 나온다. 이 모듈은 이걸 두
방식으로 막는다.

### 3-1. 현재 관절을 seed 로 IK — `stand_fallen_cup.py:1252` `ik_state_with_current_seed`
IK 를 풀 때 **현재 관절 자세를 시작 추정치(seed)**로 준다. KDL 솔버는 seed 근처로
수렴하므로, 같은 손목/팔꿈치 분기를 유지한 해를 얻는다. → descend/lift/carry
에서 손목이 갑자기 도는 것을 차단.

- 1차 seed 가 실패하면 seed 주변을 **점점 넓혀가며(0.2→0.9rad) 랜덤 seed 재시도**
  (`retries=12`). 해가 존재하는데 단일 seed 로 수렴 못하는 KDL 특성을 보완.
- 단, 채택은 **base_seed 대비 관절 편차 `max_seed_jump`(=90°) 이내** 해를 우선
  → 원래 분기 유지(wrist-flip/큰 swing 방지). 그런 해가 끝내 없을 때만 "편차 최소
  해"라도 반환(완전 실패보다 나음, 경고 로그와 함께).

### 3-2. 분기 점프 거부 가드 — `place_mouth_up_cup.py:369` `ik_state_with_current_seed`
mouth-up 쪽은 더 엄격하게, seed 대비 관절 변화가 임계치를 넘으면 **그 해를 거부**
(`None` 반환, fail-safe)한다.

```python
IK_MAX_JOINT_DELTA = 2.0  # rad
if dmax > max_joint_delta:
    return None   # '먼 분기(천장)'로 보고 거부
```

> 주석 그대로: *"set_from_ik 는 시드를 시작 추정으로만 쓸 뿐, 수치해가 멀리 튀어
> 다른 wrist 분기/elbow-up(천장) 해로 수렴할 수 있다. 그러면 PTP 가 그 먼 목표까지
> 팔을 크게 휘둘러(천장 휘젓기, 과거 반복 버그) 위험하다."*

### 3-3. 그립 롤 대칭 최소회전 선택 — `place_mouth_up_cup.py:391` `_choose_grip_roll`
2지 평행 그리퍼는 접근축 둘레 180° 대칭이라 `R` 과 `R·Rz(π)` 가 같은 그립이다.
둘 중 **손목이 덜 도는(시드에 가까운)** 쪽을 골라, 불필요한 ~210° joint_6 롤
(과거 가드에 걸려 종료되던 원인)을 피한다.

---

## 4. 플래너 역할 분담 + 보수적 속도 (플래너 레벨 방어)

세 플래너를 **상황에 맞게 골라** 쓰고, 실패 시 **폴백 체인**을 둔다.
설정: `config/moveit_py.yaml`, 런타임 파라미터: 각 노드 `__init__`.

| 플래너 | 용도 | 안전 성질 |
|---|---|---|
| **Pilz PTP** | HOME 이동, approach, insert, carry | 관절공간 단조 보간 → 휘젓지 않음, 결정적·최단 |
| **Pilz LIN** | descend / lift (수직) | Cartesian 직선 + orientation 잠금 → XY 고정, 대각선화 방지 |
| **OMPL RRTConnect** | 충돌 회피가 필요한 자유공간 이동 | collision-aware 샘플링 |

핵심 원칙들:

- **HOME 이동은 PTP 우선** — OMPL(RRTConnect)은 무작위 샘플링이라 시작 자세가
  HOME 과 멀면 "팔을 천장으로 쳐들었다 내려오는" 경로를 만들 수 있다(실로봇 사고).
  PTP 는 각 관절이 직접 보간돼 휘젓지 않는다(`stand_fallen_cup.py:2505`).
- **insert 는 LIN 이 아니라 PTP** — 과거 LIN(Cartesian 직선)이 손목 특이점을 지나
  joint_4 속도한계(31 vs 3.9)를 위반해 죽었다. seed IK 로 goal 이 현재와 가까워
  PTP 경로도 거의 직선이라 특이점을 피한다(`place_mouth_up_cup.py:662`).
- **폴백 체인**: 예) approach 는 `Pilz PTP → OMPL`, descend/lower 는
  `PTP → LIN`, lift 는 `LIN → 계단식 LIN → OMPL`. 한 플래너가 실패해도 다음으로
  넘어가되, **더 안전한 쪽을 먼저** 시도한다.
- **보수적 속도/가속 스케일** — 컵이 잡힌 채 흔들리거나 RG2 가 트립하지 않도록
  느리게 움직인다.

  ```
  OMPL : vel 0.30 / acc 0.15
  Pilz : vel 0.20 / acc 0.10
  LIN  : vel 0.10 / acc 0.06    ← 가장 느림(가장 정밀한 수직 동작)
  ```

---

## 5. 특이점(Singularity) 회피

손목 특이점을 가로지르면 관절 속도가 발산해 드라이버가 죽는다. 대응:

- **수직 상승/하강은 LIN + 현재 XY 고정(no-clamp)** — `_lift_straight_up`
  (`stand_fallen_cup.py:1336`, `place_mouth_up_cup.py:442`). XY 를 픽 위치에
  고정해 그리퍼가 피라미드 쪽으로 휘둘리지 않고, orientation 도 잠근다.
- **단일 LIN 실패 시 짧은 step 계단식 fallback** — `RETREAT_LIFT_STEP_M=0.08m`
  단위로 나눠 상승. step 이 짧을수록 특이점을 가로지를 확률이 낮아 LIN 성공률↑.
  중간에 실패해도 **그때까지 도달한 높이에서 멈춘다**(부분 상승도 컵 회피에 유효).
- **lift 가 목표 높이에 한참 못 미치면 OMPL(충돌회피)로 재시도**
  (`stand_fallen_cup.py:1810`).

---

## 6. 안전한 HOME 복귀 시퀀스 — `stand_fallen_cup.py:1385` `_return_to_session_home`

낮은 place 자세에서 곧장 HOME PTP 를 하면 EE 가 작업영역을 **대각선으로 가로질러**
옆 컵/테이블을 친다. 그래서 0→1→2 단계로 나눠 안전하게 복귀한다.

- **0단계 — 수직 상승 우선**: EE z 가 `HOME_PREREVERSE_MIN_Z`(0.25m)보다 낮으면
  먼저 `_lift_straight_up(LIFT_Z)` 로 수직으로 빠져나온다(수평 sweep 원천 차단).
- **1단계 — joint_1 pre-reverse**: `joint_1` 만 먼저 HOME 값으로 PTP. 다른 관절은
  유지 → EE z 변동 없음 → Cartesian dip(테이블로 꺼짐) 위험 없이 이후 복귀의
  관절 변화량을 줄인다. **단 EE 가 낮으면 joint_1 회전이 곧 수평 sweep 이므로
  스킵**하고 OMPL collision-aware 복귀에 위임.
- **2단계 — 전체 HOME 복귀(EE 높이로 플래너 선택)**:
  - EE 가 충분히 높으면(>`HOME_RETURN_HIGH_Z` 0.30) **Pilz PTP 먼저**
    (결정적·최단), OMPL fallback.
  - 낮은 자세면 **OMPL 먼저**(dip 회피), Pilz PTP fallback.
- **시작/종료 자세 검증**: 종료 link_6 pose 를 시작과 비교해 Δpos<5mm·Δrot<0.01
  이면 ✓, 아니면 "HOME 복귀 미완" 경고(`stand_fallen_cup.py:1508`).

---

## 7. mouth-up 뒤집기(flip) — IK 우회 안전 동작 — `place_mouth_up_cup.py:763`

컵을 mouth-down 으로 뒤집는 동작은 **순수 joint_6 ±180° 롤**이다. 이를 Cartesian
pose 로 만들어 IK 로 풀면 솔버가 "손목이 통째로 뒤집힌 다른 분기"를 반환 → 가드
거부 → OMPL 랜덤폴백이 컵을 들고 작업영역 밖으로 휘두른다(실측 joint_5 Δ=4rad).

→ **IK 를 아예 거치지 않고**, 현재 관절각의 `joint_6` 에만 ±180° 를 더한 목표
상태를 PTP 로 보낸다. 분기 모호성 0.

- **limit-safe 회전 방향**: 결과 `|joint_6|` 가 작아지는 쪽으로 180°(관절 한계
  ±2π 안쪽 유지) — `place_mouth_up_cup.py:782`.
- **단계 분할**(`FLIP_STAGES=2`): 팔은 정지하고 손목만 도는 안전 동작이라, 매끄러운
  swing 을 위해 2단계로 나눠 보낸다.
- 안착 직전 작은 bounded `PLACE_TILT_FIX_DEG`(5°) 보정으로 컵을 더 세워 안착
  성공률을 높인다(full-flip 이 아니라 손목 한계/도달성에 거의 영향 없음).

---

## 8. 사전 도달성 검사 (preflight) — `stand_fallen_cup.py:1656` `_preflight_grasp_reachable`

컵을 향해 **이동하기 전에**, approach·descend IK 해가 실제로 존재하는지 먼저
검사한다(실행·이동 없이 IK 만).

- descend 는 approach 해의 관절을 seed 로 풀어 **실제 실행 시퀀스를 모사** →
  "집어보면 풀릴 컵"을 도달불가로 오판하지 않음.
- 해가 없으면 그 컵은 **집지 않고** blacklist 에 넣어 다음 sense 에서도 제외하고,
  `/fallen_cup/unreachable` 토픽(JSON)으로 상위에 통보(`_publish_unreachable`).
- 도달 불가능한 컵을 향해 팔을 뻗다 중간에 멈추는 위험한 부분 궤적을 원천 차단.

---

## 9. 빈 안전지점 배치 — `stand_fallen_cup.py:956` `_select_safe_place_spot`

세운 컵을 아무 데나 놓지 않는다. 후보 PLACE 좌표 중 **정상 컵 · 이미 세운 컵 ·
남은 넘어진 컵 · 피라미드를 모두 회피**(회피 반경 `place_spot_avoid_radius_m`
0.09m)한 첫 빈 자리를 고른다.

- 컵과 **같은 Y 측** 작업영역 가장자리(|y| 큰)부터 검사 → cross-body 스윙 최소화.
- 같은 쪽이 다 막히면 반대쪽 폴백, 그래도 없으면 기본 PLACE.
- multi-cup 에서는 placing 충돌·재집기(self-pickup)를 막는 핵심 장치.

---

## 10. 그리퍼·인식 단계의 안전

### 10-1. 부드러운 그립
- `GRIP_FORCE`(fallen 200≈13N, mouth-up 250) — 얇은 컵이 변형/이탈하지 않게 약하게.
- `GRIP_Z_MARGIN`(0.020m) — 그립 지점에서 위로 띄워 그리퍼가 컵을 누르며 트립되는
  것 방지.
- sim 모드면 그리퍼 HW 를 아예 잡지 않고 명령을 no-op 처리(HW 사고 0).

### 10-2. 인식 안정화 / phantom 게이팅
- 멀티 샘플 수집(`MIN_SAMPLES`) + **circular mean** 으로 yaw 안정화, 30° 밖
  outlier 제거(`stand_fallen_cup.py:918`, `common.py:154`).
- mouth-up: 샘플이 `MIN_SAMPLES` 미만이면 **저신뢰 phantom 으로 보고
  `RECOVER_NONE`** 처리 → 멀쩡한 컵을 건드리지 않고 단계 정상 종료
  (`place_mouth_up_cup.py:536`).

---

## 11. 오케스트레이션·프로세스 레벨 안전

### 11-1. "한 호출 = 한 컵" 단일 액션 정책 — `outlier_cup_recovery.py`
fallen·mouth-up 모두 **한 번에 최대 한 컵만** 처리하고 종료해, agent 가 hand-eye
로 재판단 후 다시 호출한다. 과거 mouth-up 이 고정 드롭존에 엎어 놓은 컵을 자기
검출기가 다시 잡아 **무한 재집기**하던 문제를 구조적으로 제거(`outlier_cup_recovery.py:84`).

- 안전 상한: `MOUTH_UP_MAX_ITERATIONS=10`, fallen `multi_cup_max_iterations`.

### 11-2. 깨끗한 종료코드 전파 — `os._exit`
MoveItPy 의 C++ 소멸자가 teardown 중 hang/SIGABRT(`-6`)를 내면, 컵을 이미 세우고
HOME 복귀까지 끝낸 **성공이 `failed` 로 뒤집히고** launch→서버 LaunchManager 가
task 를 'running' 에 묶어 LLM 이 다음 plan 으로 못 넘어간다.

→ 성공 확정 후 `destroy_node()`/`rclpy.shutdown()` 을 건너뛰고 `os._exit(0/1)` 로
**종료코드를 그대로 전파**한다(1회성 태스크라 리소스는 OS 가 회수).
`stand_fallen_cup.py:2681`, `outlier_cup_recovery.py:130`.

---

## 12. 방어선 한눈에 보기

| # | 레벨 | 기법 | 무엇을 막나 | 코드 |
|---|---|---|---|---|
| 1 | 위치 | 작업영역 클램프 + 바닥 하드 플로어 | 범위 이탈·테이블 충돌 | `common.py:53`, `stand_fallen_cup.py:1758` |
| 2 | 충돌 | 그리퍼/피라미드/정상컵 collision object | 장애물 클립·피라미드 붕괴 | `stand_fallen_cup.py:2276·2340·2418` |
| 3 | 자세 | IK seed 잠금 + 분기 가드 | 손목뒤집힘·천장 휘젓기 | `stand_fallen_cup.py:1252`, `place_mouth_up_cup.py:369` |
| 4 | 플래너 | PTP/LIN/OMPL 역할분담 + 보수적 속도 | 휘젓기·대각선화·트립 | `*_cup.py __init__`, `moveit_py.yaml` |
| 5 | 특이점 | LIN 수직 + 계단식 fallback | 관절속도 발산·드라이버 사망 | `stand_fallen_cup.py:1336` |
| 6 | 실행 | 안전 HOME 복귀(수직→pre-reverse→복귀) | 수평 sweep·Cartesian dip | `stand_fallen_cup.py:1385` |
| 7 | 실행 | flip IK 우회(순수 joint_6) | 분기 모호성·큰 swing | `place_mouth_up_cup.py:763` |
| 8 | 실행 | 사전 도달성 검사 | 도달불가 컵 부분 궤적 | `stand_fallen_cup.py:1656` |
| 9 | 배치 | 빈 안전지점 선택 | 배치 충돌·self-pickup | `stand_fallen_cup.py:956` |
| 10 | HW | 부드러운 그립 + phantom 게이팅 | 컵 변형/이탈·오검출 동작 | `*_cup.py`, `place_mouth_up_cup.py:536` |
| 11 | 프로세스 | 단일 액션 + os._exit 종료코드 | 무한 재집기·상태 꼬임 | `outlier_cup_recovery.py` |

---

*생성: 2026-06-17 · 근거: 위 코드 경로의 실제 구현/주석. 상수값은 코드 기준이며
튜닝되면 같이 갱신할 것.*
