# dsfranka

DualSense 게임패드로 Franka 로봇팔을 텔레오퍼레이션하고 시연 데이터를 수집하는 프로젝트.
MuJoCo 시뮬레이션에서 조작감을 개발/검증한 뒤, 같은 코드 경로로 실제 Franka(libfranka)를 구동한다.

```
DualSense ──▶ TeleopSession (Python, 50 Hz)          ┌─ sim ──▶ MuJoCo physics (position servo)
 (pydualsense) · 스틱/버튼 → EE 타겟 pose 적분        │
               · 상태머신 (호밍/틸트/자동하강/녹화)     ├─ real ─▶ DiffIK (kinematic MuJoCo model)
               · EpisodeRecorder → HDF5              │           └▶ UDP → franka_bridge (C++, 1 kHz)
               · DiffIK: 공통 damped-least-squares   ┘                    └▶ libfranka JointPositions
```

핵심 설계: **차분 IK와 텔레옵 로직이 시뮬/실물에서 완전히 동일**하다.
시뮬은 MuJoCo 물리로 실행되고, 실물은 같은 MuJoCo 모델을 기구학 전용으로 사용해
IK를 풀고 관절 타겟을 C++ 브리지로 스트리밍한다(브리지가 1 kHz 스무딩/추종).

## 🎥 Demo

[![Watch the demo](https://img.youtube.com/vi/eDtdNF7Izfw/maxresdefault.jpg)](https://youtu.be/eDtdNF7Izfw)
**DualSense teleoperation & demonstration data collection for Franka (FR3) — MuJoCo sim + libfranka**

## 시작하기

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e .

# DualSense 접근 권한 (햅틱/hidraw, 1회)
sudo apt install -y libhidapi-hidraw0
sudo tee /etc/udev/rules.d/70-dualsense.rules <<'EOF'
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0660", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# 하드웨어 체크
python scripts/test_gamepad.py     # 입력 매핑 확인
python scripts/test_haptics.py    # 라이트바/진동/트리거/IMU 확인

# 텔레옵 (뷰어 필요 — 데스크탑 세션에서 실행)
python scripts/teleop_sim.py

# 패드/화면 없이 스모크 테스트
python scripts/teleop_sim.py --mock --headless --ticks 200
python tests/test_pipeline.py     # 통합 테스트
```

## 조작법

| 입력 | 기능 |
|---|---|
| 왼쪽 스틱 | x-y 평면 이동 |
| L1 / R1 | z 상승 / 하강 (누르는 동안 — 왼스틱과 함께 "내려가며 이동" 가능) |
| 오른쪽 스틱 ←→ | yaw 회전 (오른쪽 = 시계방향, 상하축은 미사용) |
| D-pad (누른 채) | 틸트 성분·방향 지정 (↑↓=상하, ←→=좌우) — **누르고 있는 동안에만 ✕/○가 동작** |
| D-pad+✕ 탭 / 꾹 | 지정 방향으로 **현재 값에서** 30° 스텝(눈금 스냅, 74°→90°) / 연속 미세 틸트(40°/s) |
| D-pad+○ 탭 / 꾹 | 해당 성분을 0° 쪽으로 30° 스텝(74°→60°) / 연속 복귀 (0°에서 정지) |
| **✕ / ○ 단독** | **그리퍼 닫기 / 열기 (주 조작)** — D-pad를 안 누른 상태에서 |
| ▢ (Square) | orientation 리셋 (yaw·틸트 → 0, 위치 유지) |
| △ (Triangle) | 호밍 (홈 EE pose로 복귀) |
| R3 (오른스틱 클릭) | 지정 높이까지 자동 하강 |
| R2 | 그리퍼 토글 (보조, 디텐트 햅틱) — 80% 넘게 누르면 닫힘, 20% 아래로 놓으면 열림. 임계 "교차" 기준이라 ✕/○로 정한 상태를 방치된 트리거가 뒤집지 않음 |
| Create (촬영키) | 에피소드 녹화 시작 / 정지+저장 |
| Options (메뉴키) | 녹화 중이면 폐기 |
| PS | 세션 종료 |

- **조작 기준** `control.operator_position`: 기본 `front`는 로봇을 마주보는 기준
  (스틱 x-y·틸트 방향 미러링). 로봇 뒤에서 조작하면 `behind`. yaw는 어느 기준이든 동일.
- **틸트**는 상하/좌우 두 성분(각 ±90°)의 합성. D-pad로 성분을 오가도 값이 이어진다 —
  전방 60° 상태에서 ↓+✕ → 30°. 복합 틸트(전방+측면) 가능, ▢가 전체 리셋.
- 자동 이동(호밍/하강) 중 스틱·L1/R1 조작 시 즉시 취소.
- 모든 파라미터(속도, 데드존, 워크스페이스, 틸트 스텝/최대각)는 `configs/teleop.yaml`.

**햅틱 피드백** (pydualsense 드라이버 기본; `gamepad.driver: evdev`는 입력 전용 폴백):

| 채널 | 신호 | 의미 |
|---|---|---|
| 고주파 모터 | 타겟 ↔ 워크스페이스 박스 면 거리 | 경계 접근 경고 (5 cm부터 램프) |
| 저주파 모터 | IK anti-windup \|q_ref − q\| 포화율 | 서보가 세게 밀고 있음 (누르기/막힘) |
| R2 트리거 (디텐트) | 토글 임계점 접근 | 약한 프리로드(0~60%) → 강한 벽(60~80%) → 관통 시 저항 소멸 = 딸깍. 해제 벽은 40~20% (sticky 히스테리시스로 떨림 방지) |
| 라이트바 | 녹화 상태 | 빨강=녹화 중, 파랑=대기 |

## 제어 구조와 로드맵

**현재 제어 모드: 관절 위치 제어 (Stage 0).** 액션(EE pose 타겟)의 오차→토크 변환은
일어나지만, 관절 공간에서 고정된 고강성으로 서보 내부에 숨어 있다:

```
액션(EE pose, 50 Hz) → DiffIK: cartesian 오차 → q_ref
  시뮬:  MuJoCo position 서보   τ = 4500·(q_ref−q) − 450·q̇
  실물:  libfranka JointPositions → Franka 내부 관절 임피던스 (1 kHz)
```

| Stage | 제어 모드 | 오차→토크 위치 | 변경 범위 | 상태 |
|---|---|---|---|---|
| 0 | joint position | 서보 내부 (암묵적, 고강성) | — | ✅ 시뮬 완성 |
| 1 | 〃 (실물 브링업) | 〃 (`setJointImpedance`로 강성만 조절 가능) | 없음 | 실물 검증 대기 |
| 2 | joint impedance (토크 콜백) | **브리지 코드** τ = Kp(q_ref−q) − Kd·q̇ | 브리지 내부만 (프로토콜 유지) | 예정 |
| 3 | **cartesian impedance** | τ = Jᵀ[K_x(x_action ⊖ x_obs) + D_x·ẋ오차] + 널스페이스 | 프로토콜 v2 (EE pose 전송, 외력 기록) + 시뮬 torque 액추에이터 전환 | 목표 |
| 4 | 정책 배포 | 정책 액션(EE pose)을 Stage 3 제어기가 추종 | — | 최종 |

Stage 3에서 기록된 **action−obs 오차가 곧 렌치(N/Nm)**가 되고 K_x로 접촉 강성을 직접
정의한다. 접촉 태스크(누르기/끌기) 데이터는 Stage 3 전환 후 수집해야 배포 시
동역학이 일치한다.

### Stage 3 구현 레퍼런스: robosuite OSC (LIBERO가 사용)

[robosuite `controllers/parts/arm/osc.py`](https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/controllers/parts/arm/osc.py)
— 소스에서 검증한 핵심 메커니즘:

```
goal = 측정된 현재 EE pose + clip(delta)         # 목표를 관측에 재앵커링 ('achieved' 모드)
F    = kp·(goal − x_obs) + kd·(ẋ_d − ẋ)         # 오차가 곧 힘 (도달/정지 로직 없음)
τ    = Jᵀ·(Λ·F) + 중력보상 + 널스페이스 토크      # Λ = (J·M⁻¹·Jᵀ)⁻¹ 동역학 디커플링
kd   = 2·√kp·ζ                                   # ζ=1 임계감쇠 → 튜닝 자유도는 kp 하나
```

가져올 포인트:

1. **오차 클램프 = 힘 상한**: 델타가 스텝당 최대 5 cm로 클립되고 목표가 매번 관측
   pose에 재앵커링되므로 스프링 최대 신장 = 5 cm → 최대 힘 ≈ kp×0.05 (kp=150이면
   ~7.5 N). 와인드업 방지가 액션 설계에 내장 — 우리 브리지에서는
   `clamp(x_action ⊖ x_obs)` 한계값이 곧 최대 접촉력(N)이 되도록 설계한다.
2. **임계감쇠 규칙** kd = 2√kp·ζ (ζ=1) — 게인 출발점 kp≈150.
3. **분리 원칙**: 중력보상은 스프링 밖에서 (Franka 토크 모드는 자동), 코리올리스는
   `franka::Model`, 여유 자유도는 널스페이스 토크로 홈 자세 유지.
   libfranka의 `cartesian_impedance_control` 예제가 동일 구조.
4. **위치와 힘이 한 법칙**: 자유 공간에서는 추종(트레일링 = 스프링 신장), 접촉 시
   같은 오차가 누르는 힘이 된다. LIBERO가 접촉 시연을 수집할 수 있는 이유이며,
   시연·배포가 같은 오차→힘 경로를 타는 것이 데이터 일관성의 핵심.

**조작감(부드러움) 필터** — LIBERO OSC 임피던스의 역할을 위치 파이프라인에서 재현:

1. 타겟 가속 제한 (`accel.xyz/yaw`) — 스틱 입력을 계단이 아닌 램프로
2. 자동 이동 사다리꼴 감속 — 목표 앞 √(2·a·d) 감속
3. IK 태스크공간 속도 클램프 + 완화 게인 (`ik.max_lin_vel`, `pos_gain`)

EE가 타겟을 뒤따르는 트레일링(≈속도/pos_gain)은 의도된 특성.
더 기민하게: `pos_gain`·`accel` ↑. 실물은 브리지가 관절 속도 한계의 40%로 추가 클램프.

## 실제 로봇 (libfranka)

설치/빌드/실습 순서는 [cpp/README.md](cpp/README.md) 참고. 요약:

```bash
./cpp/build/franka_bridge <robot-ip>   # RT 커널 권장, 로봇은 FCI 모드
python scripts/teleop_real.py
```

브리지는 현재 관절각과 가까운 첫 명령을 받을 때까지 대기하므로(클라이언트가 로봇
상태로 동기화 후 전송) 급격한 초기 이동이 없다.

**홈 자세 프리셋** (`configs/teleop.yaml` → `home.select`) — 홈(△ 버튼)·IK 널스페이스
기준·초기 타겟에 쓰인다. 세 프리셋 제공:

| `home.select` | 자세 | 출처 |
|---|---|---|
| `franka_ready` | q4=−3π/4 | Franka 내장 "ready" (실물 부팅/Desk 이동 위치) |
| `dsfranka` (기본) | q4=−π/2 | menagerie fr3_hand 키프레임 (시뮬 모델 일치) |
| `libero` | q2=π/16, q6=π−0.2 | robosuite/LIBERO Panda init_qpos (**본인 LIBERO 버전과 대조 검증 필요**) |

실물은 텔레옵 시작 전 로봇이 선택한 홈에 물리적으로 있어야 한다(`dsfranka`는
`02_move_to_home`, `franka_ready`는 부팅 자세라 이동 불필요).

**로봇 타입 (FR3 기본)** — 두 곳이 짝을 이룬다:

| 위치 | 설정 | 효과 |
|---|---|---|
| `configs/teleop.yaml` → `robot: fr3` | Python/시뮬 | MuJoCo 모델·IK 관절한계 (`assets/franka_fr3/`) |
| `cmake -B build -DDSFRANKA_ROBOT=fr3` | C++ 브리지 | 컴파일 타임 관절 위치/속도 한계 |

FR3 모델은 menagerie `franka_fr3` + Franka Hand를 결합한 `fr3_hand.xml`
(홈 TCP pose가 Panda 모델과 일치함을 검증). Panda는 `robot: panda` + 재빌드.

**워크스페이스 캘리브레이션 (kinesthetic)** — 브리지를 켜둔 채 로봇을 핸드가이딩으로
원하는 범위를 훑은 뒤:

```bash
python scripts/calibrate_workspace.py --duration 60          # 미리보기
python scripts/calibrate_workspace.py --duration 60 --write  # 적용
```

결과는 `configs/workspace_calibrated.yaml`로 저장되어 로드 시 teleop.yaml의 workspace를
대체한다(삭제로 원복). 누르기/끌기용으로 z 하한이 음수여도 무방.

## 에피소드 포맷 (HDF5)

```
/t                  (N,)   timestamp
/obs/q, /obs/dq     (N,7)  관절 위치/속도
/obs/ee_pos         (N,3)  TCP 위치
/obs/ee_quat        (N,4)  TCP orientation (wxyz)
/obs/gripper_width  (N,)
/action/ee_pos      (N,3)  타겟 pose (액션, 절대값 — 델타 변환은 후처리)
/action/ee_quat     (N,4)
/action/gripper     (N,)   0(닫힘)~1(열림)
```

## 레포 구조

```
assets/                  MuJoCo 모델: franka_fr3 (fr3+hand 결합, 기본) / franka_emika_panda
configs/teleop.yaml      매핑/속도/IK/워크스페이스/햅틱 설정
src/dsfranka/
  common/                types, DiffIK(공통 IK), config 로더, Rate
  input/                 pydualsense 드라이버(햅틱·IMU) / evdev 폴백 / Mock
  teleop/                TeleopSession (상태머신), FeedbackController (햅틱 정책)
  sim/                   MujocoArm 백엔드
  real/                  FrankaArm 백엔드 (UDP 클라이언트, 미검증)
  data/                  EpisodeRecorder (HDF5)
scripts/                 teleop_sim / teleop_real / calibrate_workspace / test_*
cpp/
  examples/              libfranka 실습 예제 (읽기 → 홈 이동)
  bridge/                franka_bridge (UDP ↔ 1 kHz), robot_limits.hpp, udp_protocol.hpp
tests/test_pipeline.py   헤드리스 통합 테스트
data/episodes/           수집된 에피소드 (episode_NNNN.hdf5)
```

## 기능 로드맵

제어 모드 로드맵(Stage 0→4)은 위 표 참고. 기능 단위:

- [x] MuJoCo 텔레옵 + 에피소드 녹화 / FR3 고정 / 햅틱 피드백 / 워크스페이스 캘리브레이션 도구
- [ ] 실물 franka_bridge 검증 (Stage 1)
- [ ] 프로토콜 v2: EE pose 전송 + `O_F_ext_hat_K` 외력 기록 (Stage 3 선행 작업)
- [ ] 카메라 관측 녹화 (RealSense) → LeRobot 포맷 변환
- [ ] IMU 활용 (패드 기울여 미세 틸트, 흔들어 폐기 등 — GamepadState에 노출됨)
- [ ] LICENSE 추가 (조작감 확정 후)

## Credits

- 로봇 모델: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)의
  `franka_fr3` / `franka_emika_panda` (각 디렉토리의 LICENSE 참고). `fr3_hand.xml`은
  두 모델을 결합한 파생본이며 TCP site가 추가되어 있다.
- 텔레옵 데이터 수집 설계 레퍼런스: [LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) /
  [robosuite](https://github.com/ARISE-Initiative/robosuite) (OSC_POSE 컨트롤러 — Stage 3 구현 레퍼런스)
