# libfranka 실습 & 브리지

## 0. 사전 조건

- 제어 PC ↔ 로봇 직결 이더넷, Desk 접속 가능 (`https://<robot-ip>`)
- Desk에서 FCI 활성화 (Settings → 잠금 해제 → Activate FCI)
- **PREEMPT_RT 커널 강력 권장** (없으면 `franka::Robot(ip, franka::RealtimeConfig::kIgnore)` 필요, 지터로 reflex 정지 잦음)
- 로봇 시스템 버전에 맞는 libfranka 버전 확인
  (FR3 → libfranka 0.10+, Panda(FER) 4.2.x → libfranka 0.9.2, 4.0/4.1 → 0.8.0)

## 1. libfranka 설치 (소스 빌드)

```bash
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
git clone --recurse-submodules https://github.com/frankaemika/libfranka.git  # 버전 태그 checkout
cd libfranka && git checkout <version-tag> && git submodule update
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
cmake --build build -j
# 시스템 설치(선택): sudo cmake --install build
```

## 2. 이 레포 빌드

```bash
cd cpp
cmake -B build -DFranka_DIR=<libfranka>/build   # 시스템 설치했다면 생략 가능
# 로봇 타입은 FR3가 기본. Panda면: cmake -B build -DDSFRANKA_ROBOT=panda ...
cmake --build build -j
```

로봇별 관절 위치/속도 한계는 `bridge/robot_limits.hpp`에 컴파일 타임으로 고정되며
(`configs/teleop.yaml`의 `robot:`과 짝 유지), 브리지가 위치 한계(마진 0.02 rad)와
속도 한계(공식 한계의 40%)로 모든 명령을 클램프한다.

## 3. 실습 순서

| 단계 | 실행 | 내용 |
|---|---|---|
| 1 | `./build/01_echo_state <ip>` | 연결 + 상태 읽기 (모션 없음, 안전) |
| 2 | `./build/02_move_to_home <ip>` | ⚠️ 로봇 이동: cosine 보간 관절 P2P |
| 3 | `./build/franka_bridge <ip>` | 텔레옵 브리지 (Python 클라이언트와 연동) |

각 단계 전에 작업공간 확보, user stop(외부 활성화 장치) 소지 필수.

## 4. franka_bridge 동작

- UDP :5555 로 `CmdPacket`(관절 타겟 7 + 그리퍼 + 플래그) 수신, 최신 값만 유지
- 시작 시 `readOnce()`로 상태만 퍼블리시하며 대기 → **현재 관절각과 0.05 rad 이내의
  첫 명령이 와야** 1 kHz 제어 시작 (Python 클라이언트가 로봇 상태로 동기화 후 전송)
- 1 kHz `JointPositions` 콜백: 지수 스무딩 + rad/tick 클램프로 저주파 명령을 추종
- 상태(`StatePacket`)를 UDP :5556 으로 100 Hz 퍼블리시
- 그리퍼는 별도 스레드에서 `franka::Gripper` blocking 호출 (임계값 넘는 변화만 반영)

패킷 레이아웃은 `bridge/udp_protocol.hpp` ↔ `src/dsfranka/real/franka_client.py` 동기 유지.

## 5. 트러블슈팅

- `communication_constraints_violation` reflex → RT 커널/CPU isolation 확인, 같은 PC에서 무거운 프로세스 제거
- `Motion finished commanded, but ...` / reflex 정지 → Desk에서 에러 복구 후 브리지 재시작
- 연결 자체가 안 되면 → FCI 활성화 여부, 방화벽, 로봇 IP 확인
