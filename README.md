# dsfranka

DualSense 게임패드로 Franka 로봇팔을 텔레오퍼레이션하고 시연 데이터를 수집하는 프로젝트.
MuJoCo 시뮬레이션에서 조작감을 개발/검증한 뒤, 같은 코드 경로로 실제 Franka(libfranka)를 구동한다.

## 아키텍처

```
DualSense ──▶ TeleopSession (Python, 50 Hz)          ┌─ sim ──▶ MuJoCo physics (position servo)
  (evdev)      · 스틱/버튼 → EE 타겟 pose 적분        │
               · 상태머신 (호밍/틸트/자동하강/녹화)     ├─ real ─▶ DiffIK (kinematic MuJoCo model)
               · EpisodeRecorder → HDF5              │           └▶ UDP → franka_bridge (C++, 1 kHz)
               · DiffIK: 공통 damped-least-squares   ┘                    └▶ libfranka JointPositions
```

핵심 설계: **차분 IK와 텔레옵 로직이 시뮬/실물에서 완전히 동일**하다.
시뮬은 MuJoCo 물리로 실행되고, 실물은 같은 MuJoCo 모델을 기구학 전용으로 사용해
IK를 풀고 관절 타겟을 C++ 브리지로 스트리밍한다(브리지가 1 kHz 스무딩/추종).

## 조작 매핑

| 입력 | 기능 |
|---|---|
| 왼쪽 스틱 | x-y 평면 이동 (베이스 프레임) |
| 오른쪽 스틱 ↑/↓ | z 높이 연속 제어 |
| L1 / R1 | yaw 회전 +/− |
| D-pad | 틸트 방향 선택 (↑=전방, ↓=후방, ←=좌, →=우로 기울임) |
| ✕ (Cross) 탭 | 틸트 +30° 스텝 — 어중간한 각도(예: 74°)에선 위 눈금(90°)으로 스냅 |
| ✕ (Cross) 꾹 | 연속 미세 틸트 증가 (기본 40°/s, 0.35s 이상 누르면) |
| ○ (Circle) 탭 | 틸트 −30° 복귀 — 어중간한 각도에선 아래 눈금(60°)으로 스냅 |
| ○ (Circle) 꾹 | 연속 미세 복귀 |
| ▢ (Square) | orientation 리셋 (yaw·틸트 → 0, 위치 유지) |
| △ (Triangle) | 호밍 (홈 EE pose로 복귀) |
| R3 (오른스틱 클릭) | 지정 높이까지 자동 하강 |
| L2 / R2 | 그리퍼 열기 / 닫기 (아날로그) |
| Create (촬영키) | 에피소드 녹화 시작 / 정지+저장 |
| Options (메뉴키) | 녹화 중이면 폐기 |
| PS | 세션 종료 |

틸트는 최대 90°(설정 가능)이며 D-pad로 방향을 바꾸면 0°로 복귀 후 새 방향이 활성화된다.
자동 이동(호밍/틸트/하강) 중 스틱·L1/R1을 조작하면 즉시 취소된다.
매핑 파라미터(속도, 데드존, 워크스페이스, 하강 높이, 틸트 스텝/최대각/홀드 속도)는 `configs/teleop.yaml`.

## 로봇 타입 선택 (FR3 / Panda)

이 레포는 **FR3 기본**으로 고정되어 있다. 두 곳이 짝을 이룬다:

| 위치 | 설정 | 효과 |
|---|---|---|
| `configs/teleop.yaml` → `robot: fr3` | Python/시뮬 | MuJoCo 모델·IK 관절한계 선택 (`assets/franka_fr3/`) |
| `cmake -B build -DDSFRANKA_ROBOT=fr3` (기본값) | C++ 브리지 | 컴파일 타임 관절 위치/속도 한계 (`cpp/bridge/robot_limits.hpp`) |

FR3 모델은 menagerie `franka_fr3`(관절한계 FR3 실측값)에 Franka Hand를 결합한
`fr3_hand.xml`을 사용하며, 홈 TCP pose가 Panda 모델과 정확히 일치함을 검증했다.
Panda로 바꾸려면 yaml에서 `robot: panda`, 브리지는 `-DDSFRANKA_ROBOT=panda`로 재빌드.

속도/부드러움: 스틱 수동 속도(`speed.xy` 등)와 자동 이동 속도(`speed.auto_xyz/auto_rot`)가
분리되어 있다. 부드러운 모션을 위해 세 겹의 필터가 있다 (LIBERO/robosuite의 OSC 임피던스가
하는 역할을 위치 기반 파이프라인에서 재현):

1. **타겟 가속 제한** (`accel.xyz/yaw`) — 스틱 입력이 계단이 아닌 램프로 반영
2. **자동 이동 사다리꼴 감속** — 목표 앞에서 √(2·a·d)로 감속, 급정지 없음
3. **IK 태스크공간 속도 클램프 + 완화된 게인** (`ik.max_lin_vel`, `pos_gain` 등)

EE가 타겟을 약간 뒤따라오는 트레일링(≈ 속도/pos_gain)은 의도된 것이다. 더 기민하게:
`pos_gain`↑, `accel`↑. 더 부드럽게: 반대로. 실물에서는 브리지가 로봇별 관절 속도 한계의
40%로 추가 클램프한다.

## 워크스페이스 캘리브레이션 (kinesthetic)

텔레옵 타겟은 `configs/teleop.yaml`의 `workspace` 박스로 클램프된다. 실물에서 이 박스를
직접 정하려면: 브리지를 켜둔 채(제어 시작 전 대기 상태에서도 상태를 퍼블리시함) 로봇을
핸드가이딩으로 원하는 범위 전체를 훑는다.

```bash
python scripts/calibrate_workspace.py --duration 60          # 미리보기 (dry run)
python scripts/calibrate_workspace.py --duration 60 --write  # 적용
```

결과는 `configs/workspace_calibrated.yaml`로 저장되며, 존재하면 로드 시점에 teleop.yaml의
workspace를 대체한다(파일을 지우면 원복). z 하한이 음수여도 무방하다 — 베이스 평면 아래로
누르기/끌기 작업을 할 거면 스윕에 그 높이를 포함시키면 된다.

## 레포 구조

```
assets/                  MuJoCo 모델: franka_fr3 (fr3+hand 결합, 기본) / franka_emika_panda
configs/teleop.yaml      매핑/속도/IK/워크스페이스 설정
src/dsfranka/
  common/                types, DiffIK(공통 IK), Rate
  input/                 DualSense evdev 드라이버, Mock
  teleop/                TeleopSession (상태머신)
  sim/                   MujocoArm 백엔드
  real/                  FrankaArm 백엔드 (UDP 클라이언트, 미검증)
  data/                  EpisodeRecorder (HDF5)
scripts/                 teleop_sim.py / teleop_real.py / test_gamepad.py
cpp/
  examples/              libfranka 실습 예제 (읽기 → 홈 이동)
  bridge/                franka_bridge (UDP↔1kHz 관절 제어)
data/episodes/           수집된 에피소드 (episode_NNNN.hdf5)
```

## 시작하기 (시뮬)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e .

# DualSense 확인 (USB 또는 블루투스 연결 후)
python scripts/test_gamepad.py

# 텔레옵 (뷰어 필요 — 데스크탑 세션에서 실행)
python scripts/teleop_sim.py

# 패드/화면 없이 스모크 테스트
python scripts/teleop_sim.py --mock --headless --ticks 200
```

DualSense 권한 문제 시: `sudo usermod -aG input $USER` 후 재로그인.

## 실제 로봇 (libfranka)

설치/빌드/실행 절차는 [cpp/README.md](cpp/README.md) 참고. 요약:

```bash
# 1. 브리지 실행 (RT 커널 권장, 로봇은 FCI 모드)
./cpp/build/franka_bridge <robot-ip>
# 2. 텔레옵 클라이언트
python scripts/teleop_real.py
```

브리지는 시작 시 현재 관절각과 가까운 첫 명령을 받을 때까지 대기하므로
(클라이언트가 로봇 상태로 동기화 후 전송) 급격한 초기 이동이 없다.

## 에피소드 포맷 (HDF5)

```
/t                  (N,)   timestamp
/obs/q, /obs/dq     (N,7)  관절 위치/속도
/obs/ee_pos         (N,3)  TCP 위치
/obs/ee_quat        (N,4)  TCP orientation (wxyz)
/obs/gripper_width  (N,)
/action/ee_pos      (N,3)  타겟 pose (액션)
/action/ee_quat     (N,4)
/action/gripper     (N,)   0(닫힘)~1(열림)
```

## 로드맵

- [x] MuJoCo 텔레옵 + 에피소드 녹화
- [x] FR3 모델/한계 고정 (config + cmake)
- [x] kinesthetic 워크스페이스 캘리브레이션 도구 (실물 검증 필요)
- [ ] 실물 franka_bridge 검증 (joint position streaming)
- [ ] **외력 측정** — `O_F_ext_hat_K`(추정 외력 렌치)를 StatePacket에 추가하고 에피소드
      obs로 녹화 (누르기/끌기 작업용; `cpp/bridge/udp_protocol.hpp`의 TODO 참고)
- [ ] 브리지 cartesian impedance(토크) 제어로 업그레이드 — 접촉 작업 안정성
- [ ] 카메라 관측 녹화 (RealSense) → LeRobot 포맷 변환
- [ ] DualSense 럼블/LED 피드백 (녹화 상태 표시, 충돌 경고)
