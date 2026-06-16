# 특이점(Singularity)과 IK 가드 — 심화 설명

`safety_trajectory_control.md` 의 §3(IK seed 잠금/분기 가드)·§5(특이점 회피)를
"개념부터" 풀어 쓴 심화 문서. **특이점이 무엇인지 모르는 상태**를 가정하고,
비유 → 원리 → 이 코드의 처리 순서로 설명한다.

대상 코드:
- `stand_fallen_cup.py` : `ik_state_with_current_seed`(1252), `_lift_straight_up`(1336)
- `place_mouth_up_cup.py` : `ik_state_with_current_seed`(369), `_choose_grip_roll`(391),
  insert(662), flip(763)
- 로봇 관절한계: `ros2-cup-stack/.../dsr_moveit_config_m0609/config/joint_limits.yaml`

---

## 1부. 특이점(Singularity)이 뭔가

### 1-1. 한 줄 정의

> **특이점 = 로봇 손끝(EE)을 어떤 방향으로 아주 조금 움직이려는데, 그걸 만들려면
> 어떤 관절이 "무한히 빠르게" 돌아야 하는 자세.**

손끝은 천천히 움직이는데 관절은 미친 듯이 빨라야 하는 **순간**이 있다. 그 자세가
특이점이다.

### 1-2. 일상 비유 3개

**(a) 세계지도의 북극**
위도·경도로 위치를 표현할 때, 적도에서는 경도 1° 차이가 약 111km다. 그런데
북극에 가까워질수록 경도선이 한 점으로 모여서, 북극 바로 옆에서는 "경도를 180°
바꿔도" 실제 거리는 몇 cm다. 반대로 말하면 **북극점에서 동쪽으로 1m 가려면 경도를
거의 무한대로 돌려야** 한다. 북극점이 위도·경도 좌표계의 *특이점*이다.
로봇의 특이점도 똑같다: 어떤 자세에서 "EE를 1cm 옆으로" 옮기려면 "관절을 거의
무한대로" 돌려야 한다.

**(b) 팔을 쭉 폈을 때 (팔꿈치 특이점)**
사람 팔을 앞으로 **완전히 쭉 편** 상태를 생각하자. 이때 손을 "몸에서 더 멀리"
내밀고 싶어도 더 못 나간다(이미 최대로 폄). 손을 "아주 조금만 더 멀리"
보내려는 그 경계에서, 팔꿈치는 어느 방향으로 굽혀야 할지 애매해지고 작은 손
움직임에 팔꿈치가 격렬하게 반응한다. 이게 **elbow singularity**다.

**(c) 짐벌락 / 손목 정렬 (손목 특이점)**
손목의 두 회전축(예: joint_4 와 joint_6)이 **일직선으로 정렬**되면, 두 관절이
"같은 회전"을 만들어 1 자유도가 사라진다. 이 근처에서 EE 방향을 살짝 틀려고 하면
joint_4 와 joint_6 가 서로 상쇄하며 **엄청난 속도로** 돌아야 한다. 이게 6축
로봇에서 가장 흔히 터지는 **wrist singularity**이고, 이 모듈에서도 사고를 낸 종류다.

### 1-3. 조금 더 정확히 — 야코비안(Jacobian)

로봇은 "관절을 이만큼 돌리면 손끝이 이만큼 움직인다"는 관계를 가진다. 이 변환을
행렬로 쓴 것이 **야코비안 `J`** 다.

```
손끝 속도(EE velocity)  =  J × 관절 속도(joint velocity)
```

우리가 원하는 건 보통 반대다 — "손끝을 이 방향으로 움직이고 싶다 → 관절을 얼마나
돌려야 하나?" 그래서 역으로 푼다:

```
관절 속도  =  J⁻¹ × 손끝 속도
```

**특이점은 `J` 의 역행렬 `J⁻¹` 이 발산(→∞)하는 자세**다. 수학적으로는 `det(J)=0`
(행렬식이 0). 손끝 속도가 작아도 `J⁻¹` 가 거대하면 관절 속도가 폭발한다.
→ 이게 "손끝은 천천히, 관절은 무한대로"의 정체다.

### 1-4. 왜 위험한가 — 실제 사고로 연결

관절에는 **속도 한계**가 있다. M0609 의 실제 한계(`joint_limits.yaml`):

| 관절 | max_velocity (rad/s) |
|---|---|
| joint_1 | 2.618 |
| joint_2 | 2.618 |
| joint_3 | 3.14 |
| joint_4 | **3.927** |
| joint_5 | 3.927 |
| joint_6 | 3.927 |

특이점을 가로지르는 직선 궤적을 실행하려 하면, 그 순간 필요한 관절 속도가
한계를 **수 배~수십 배** 초과한다. 그러면:

1. 플래너/컨트롤러가 "속도 한계 위반"으로 궤적을 거부하거나,
2. 실행 중이면 드라이버가 보호정지(SIGABRT·`-6`)를 내고 죽는다.

→ 컵을 놓치고, 태스크가 'failed'로 꼬이고, 최악엔 팔이 급발진하듯 움직인다.

### 1-5. 이 코드에서 **실제로 터졌던** 특이점 사고

`place_mouth_up_cup.py:662` insert 주석에 사고가 그대로 적혀 있다:

> *"과거 LIN(Cartesian 직선)이 손목 특이점을 지나 joint_4 속도한계(31 vs 3.9)를
> 위반해 죽었다."*

- **31 rad/s** = 직선 궤적이 손목 특이점을 지나는 순간 joint_4 에 요구된 속도
- **3.9 rad/s** = joint_4 의 실제 한계(위 표의 3.927)
- → 약 **8배 초과** → 드라이버 사망.
- 해결: insert 를 **LIN(직선) 대신 PTP(관절보간)**로 바꿈(아래 2-1에서 설명).

---

## 2부. 특이점을 다루는 5가지 코드 기법

핵심 통찰: **특이점에서 폭발하는 것은 "Cartesian(직선) 제약"이다.** 손끝이 정확한
직선/정해진 방향을 따라가도록 강제할 때 `J⁻¹` 가 발산한다. 반대로 **관절을 직접
보간(joint-space)** 하면 손끝 경로가 약간 휘어도 관절 속도는 매끄럽고 한계 안에
머문다. 5가지 기법은 전부 이 통찰의 응용이다.

### 2-1. (가장 중요) Cartesian(LIN) 대신 Joint-space(PTP)를 기본으로

| 플래너 | 보간 방식 | 특이점에서 |
|---|---|---|
| **Pilz LIN** | EE 가 직선을 따라가도록 강제(Cartesian) | `J⁻¹` 발산 → 관절속도 폭발 |
| **Pilz PTP** | 관절각을 직접 보간(joint-space) | 발산 없음(관절을 직접 명령) |
| **OMPL RRTConnect** | 충돌 없는 관절 경로를 샘플링 | 발산 없으나 경로가 과할 수 있음 |

그래서 approach·insert·carry·HOME 등 **대부분의 이동을 PTP로** 한다. LIN 은
"꼭 직선이어야 하는" 수직 상승/하강에만 제한적으로 쓴다(2-3).

### 2-2. seed IK 로 목표를 "현재와 가깝게" 만들어 PTP 경로를 직선화

PTP 는 관절을 보간하므로, 시작 관절과 목표 관절이 **가까우면** 손끝 경로도 거의
직선이 되고 특이점 영역을 안 건드린다. 그래서 목표 자세의 관절각을 IK 로 풀 때
**현재 관절을 seed 로** 줘서(§3에서 상세), goal 관절이 현재와 가깝게 나오도록
유도한다.

> insert 주석: *"seed IK 라 goal 관절이 현재와 가까워 PTP 경로도 거의 직선이고
> 특이점을 회피한다."* (`place_mouth_up_cup.py:662`)

### 2-3. 직선이 꼭 필요한 수직 상승/하강 — 짧은 step 계단식 LIN

컵을 든 채 수직으로 빼는 lift, 똑바로 내리는 descend 는 직선이어야 한다(옆으로
휘면 피라미드/옆컵 충돌). 그래서 LIN 을 쓰되 특이점 리스크를 이렇게 낮춘다
(`_lift_straight_up`, `stand_fallen_cup.py:1336`):

```python
# 1) 먼저 한 번의 연속 LIN 으로 target_z 까지 시도(매끄러움)
if plan_and_execute(... pose_goal=full_pose, params=self.lin_params, clamp=False):
    return target_z
# 2) 단일 LIN 실패(특이점 가로지름 등) 시에만 짧은 step 으로 fallback
step = RETREAT_LIFT_STEP_M   # 0.08 m
for i in range(n_steps):
    z_goal = min(target_z, start_z + step*(i+1))
    if plan_and_execute(... step_pose ...): reached = z_goal
    else: break               # 막힌 높이에서 멈춤(부분 상승도 유효)
```

- **`clamp=False` + 현재 XY 고정**: 목표를 "현재 XY 그대로, z만 변경"으로 줘서
  대각선화(옆으로 새는 것)를 막는다.
- **step 이 짧을수록** 한 구간이 특이점을 가로지를 확률이 낮아 LIN 성공률↑
  (대신 step 수↑ → 느려짐). 0.08m 는 그 절충값.
- **부분 성공 채택**: 중간에 막혀도 그때까지 올라간 높이는 컵/테이블 회피에 유효.

### 2-4. LIN 이 끝내 막히면 OMPL(충돌회피)로 재시도

수직 LIN 으로 목표 높이에 한참 못 미치면(특이점 등), collision-aware OMPL 로
목표 높이까지 재시도한다(`stand_fallen_cup.py:1810`). 직선은 포기하되 충돌은
피하는 경로로 높이를 확보.

### 2-5. flip 은 아예 IK 를 우회 (특이점·분기 둘 다 회피)

mouth-down 뒤집기는 "순수 joint_6 180° 롤"이다. 이걸 Cartesian pose 로 만들어 IK
로 풀면 손목이 뒤집힌 다른 분기로 수렴(특이점 근처 + 분기 점프). 그래서 IK 를
**거치지 않고** 현재 관절각의 joint_6 에만 ±180° 를 더한 목표를 PTP 로 보낸다
(`place_mouth_up_cup.py:763`). 관절을 직접 명령하니 특이점도 분기 모호성도 0.

```python
flip_dir = -1.0 if j6_0 > 0.0 else 1.0   # |joint_6| 가 작아지는 쪽(한계 ±2π 안)
stage_joints["joint_6"] = j6_0 + flip_dir * math.pi * frac   # 2단계 분할 PTP
```

---

## 3부. IK 가드 — 깊이 설명

### 3-1. IK(역기구학)와 "다중 해(분기)"

- **순기구학(FK)**: 관절각 → 손끝 위치/자세. 해가 **하나**.
- **역기구학(IK)**: 손끝 위치/자세 → 관절각. 해가 **여러 개**일 수 있다.

같은 손끝 위치라도 로봇이 취할 수 있는 자세는 여럿이다. 대표적으로:

- **elbow-up vs elbow-down**: 팔꿈치를 위로 든 자세 / 아래로 내린 자세
- **wrist-flip(손목 뒤집힘)**: 손목을 정상으로 / 180° 뒤집어서
- **shoulder left/right** 등

이 각각을 **분기(branch)**라 부른다. 같은 목표라도 어느 분기를 고르느냐에 따라
관절각이 완전히 다르다.

### 3-2. 문제의 핵심 — "올바른 위치, 위험한 분기"

이 모듈이 쓰는 KDL 수치 IK 솔버는 **seed(시작 추정 관절각) 근처로 수렴**한다.
그런데 수치해가 멀리 튀어 **엉뚱한 분기**(특히 "팔꿈치 위로/손목 뒤집힘 = 천장
분기")로 수렴할 때가 있다. 그러면:

1. IK 가 반환한 목표 관절각이 현재와 **수 rad** 떨어져 있고,
2. PTP 가 그 먼 목표까지 가려고 **팔을 천장으로 쳐들었다 내려오며 크게 휘두른다**
   (실측: joint_5 Δ=4rad).

> ⚠️ **특이점 ≠ 분기 문제.** 둘 다 "관절이 크게/빠르게 도는" 증상이지만 원인이
> 다르다. 특이점은 *Cartesian 제약 때문에* 관절속도가 발산하는 것(2부), 분기는
> *IK 가 다른 해를 골라서* 목표 자체가 멀어지는 것(3부). IK 가드는 후자를 막는다.

### 3-3. 가드 1 — 현재 관절을 seed 로 (분기 유지 유도)

`ik_state_with_current_seed` (`stand_fallen_cup.py:1252`,
`place_mouth_up_cup.py:369`): IK 를 풀 때 현재 자세를 seed 로 준다. KDL 은 seed
근처로 수렴하므로, **현재와 같은 분기**의 해를 얻을 확률이 높다 → descend/lift
에서 손목이 갑자기 도는 것을 1차로 차단.

```python
target_state.joint_positions = current_joints   # 현재 자세 = seed
target_state.set_from_ik(GROUP_NAME, pose.pose, EE_LINK, timeout)
```

### 3-4. 가드 2 — 랜덤 seed fallback + `max_seed_jump`(stand 쪽)

KDL 은 해가 존재해도 단일 seed 로는 수렴 못하는 경우가 흔하다(베이스 안쪽 + 낮은
z 의 top-down 자세 등). 그래서 1차 실패 시 **seed 주변을 점점 넓혀가며 랜덤
재시도**하되, **현재 분기에서 너무 벗어난 해는 거르도록** 임계를 둔다
(`stand_fallen_cup.py:1300`):

```python
for k in range(retries):                       # retries=12
    scale = 0.2 + 0.7*(k/(retries-1))          # 0.2 → 0.9 rad 로 점증
    seed = {jn: base_seed[jn] + uniform(-scale, scale) ...}
    cand = _solve(seed, retry_timeout)
    jump = max(abs(sol[jn] - base_seed[jn]) ...)   # 현재와의 최대 관절 편차
    if jump <= max_seed_jump:                   # = 90°
        return cand                             # 분기 유지 해 우선 채택
    best = min(best, (jump, cand))              # 아니면 '편차 최소 해' 보관
return best[1]   # 끝내 90° 이내 해가 없으면 최소 편차라도(완전 실패보다 나음)
```

- `max_seed_jump = 90°` 이내 해를 **우선** 채택 → wrist-flip/큰 swing 방지.
- 그런 해가 끝내 없을 때만 "편차 최소 해"라도 반환(경고 로그와 함께). 도달은
  하되 위험을 인지시킨다.

### 3-5. 가드 3 — 분기 점프 **거부**(mouth-up 쪽, fail-safe)

mouth-up 노드는 더 보수적이다. seed 대비 관절 변화가 임계를 넘으면 **그 해를
아예 거부**(`None`)하고 동작을 진행하지 않는다(`place_mouth_up_cup.py:381`):

```python
IK_MAX_JOINT_DELTA = 2.0   # rad
if dmax > max_joint_delta:
    return None            # '먼 분기(천장)'로 보고 거부
```

> 정상 접근/이동은 보통 관절당 1rad 안쪽이라, **2.0rad** 면 정상해는 통과하고
> "천장 휘젓기 해"만 걸러진다. 거부되면 호출부가 다른 접근방향/플래너로 폴백하거나
> 안전하게 실패 처리한다. **위험 궤적을 실행하느니 동작을 안 하는 쪽**을 택한다.

차이 정리:
- **stand**(가드 2): 거부보다는 "분기 유지 해 우선 + 최소편차 폴백" — 가급적
  동작은 시키되 안전한 분기를 고른다.
- **mouth-up**(가드 3): "임계 넘으면 즉시 거부" — flip/뒤집기 동작이 많아 분기
  점프 위험이 더 커서 더 엄격하다.

### 3-6. 가드 4 — 그립 롤 대칭 중 "덜 도는" 쪽 선택

2지 평행 그리퍼는 접근축 둘레 **180° 대칭**이라 `R` 과 `R·Rz(π)` 가 물리적으로
같은 그립이다. 두 후보 각각을 seed IK 로 풀어, **손목이 덜 도는(시드에 가까운)**
쪽을 골라 전체 시퀀스에 쓴다(`place_mouth_up_cup.py:391` `_choose_grip_roll`):

```python
for tag, Rz in (("roll0", I), ("roll180", Rz(π))):
    st, jmax, d = self._seed_ik_raw(self._flange_pose(approach_tcp, R_base@Rz))
    if d < best_d: best_R, best_d = ...     # 관절 편차 최소 쪽 채택
```

→ 불필요한 ~210° joint_6 롤(과거 가드에 걸려 종료되던 원인)을 사전에 회피.

---

## 4부. 한눈에 보기

| 구분 | 무엇 | 원인 | 이 코드의 대응 | 코드 |
|---|---|---|---|---|
| **특이점** | EE 직선 이동 시 관절속도 발산(`J⁻¹`→∞) | Cartesian 제약 | PTP 우선·seed로 직선화·계단식 LIN·OMPL 폴백·flip IK우회 | `place_mouth_up_cup.py:662`, `stand_fallen_cup.py:1336·1810`, `place_mouth_up_cup.py:763` |
| **분기 점프** | IK 가 엉뚱한 분기(천장) 해 선택 → 큰 swing | 수치 IK 의 seed 이탈 | seed=현재관절·`max_seed_jump 90°`·`IK_MAX_JOINT_DELTA 2.0` 거부·롤 대칭 최소회전 | `stand_fallen_cup.py:1252`, `place_mouth_up_cup.py:369·391` |

**핵심 한 줄**: 특이점은 *"직선을 강제하지 마라(PTP)"* 로, 분기 점프는 *"현재
자세에서 멀어지지 마라(seed 잠금 + 편차 거부)"* 로 막는다. 둘 다 본질은 **"목표
관절각을 현재와 가깝게 유지"**라는 같은 철학이다.

---

*생성: 2026-06-17 · 관절한계 수치는 M0609 `joint_limits.yaml` 기준. 동반 문서:
[`safety_trajectory_control.md`](./safety_trajectory_control.md).*
