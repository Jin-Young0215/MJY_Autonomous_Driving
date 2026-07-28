# MJY 자율주행 - 최종 주행 코드

라즈베리파이(주행 로봇)와 노트북(비전 서버) 두 프로세스가 TCP 소켓으로 통신하며 동작하는
차선 추종 + 표지판/신호 인식 자율주행 시스템.

## 아키텍처

```
[라즈베리파이: final_main.py]                     [노트북: tx.py]
  카메라 캡처 (afb.camera)
        │
        ▼
  SteeringAnalyzer (조향각 계산)
        │
        ▼
  LaneFSM (주행 시나리오 상태머신)  <───── selected label ─────  YOLO(best.pt) 추론
        │                                                          ▲
        ▼                                              JPEG frame  │
  drive_module (afb.gpio 서보/모터 제어)  ───── frame 전송 ─────────┘
```

- **final_main.py** (라즈베리파이에서 실행)
  카메라 프레임을 지속적으로 캡처하고, 차선을 분석해 조향각을 계산한 뒤
  `LaneFSM` 상태머신으로 현재 주행 모드(직진/좌우회전/정지 등)를 결정해 모터·서보를 제어한다.
  동시에 프레임을 노트북(`tx.py`)으로 스트리밍하고, 노트북이 보내주는 인식 라벨을 수신해 FSM에 반영한다.

- **tx.py** (노트북에서 실행)
  라즈베리파이로부터 프레임을 받아 YOLO(`best.pt`)로 표지판/신호를 탐지하고,
  클래스별 신뢰도·크기 규칙(`CLASS_RULES`)을 통과한 항목 중 우선순위(`PRIORITY`)가 가장 높은
  라벨 하나를 골라 라즈베리파이로 전송한다.

## 파일 구조

| 경로 | 역할 |
|---|---|
| `final_main.py` | 라즈베리파이 메인 루프. 카메라/조향/FSM/구동 제어 + 노트북과 통신 스레드 |
| `tx.py` | 노트북 메인. 프레임 수신 + YOLO 추론 + 라벨 송신 |
| `modules/steering_module.py` | `SteeringAnalyzer` — 전처리된 이진 영상에서 차선 위치·헤딩 기반 조향각 계산 |
| `modules/preprocessing_module.py` | `PreprocessingModule` — BEV(버드아이뷰) 변환, HSV 흰색 차선 이진화, 라벨링으로 노이즈 제거 |
| `modules/fsm_module.py` | `LaneFSM` — 상태 1~10단계로 구성된 주행 시나리오(출발 대기 → 주행 → 언덕 부스트 → 표지판 회전 → 게이트 대기 → 신호 대기 → 최종 회전) |
| `modules/drive_module.py` | 서보/모터 저수준 제어(`afb.gpio` 래핑) + 각도 기반 FORWARD/LEFT/RIGHT/LANE_ADJ 판정 |
| `modules/flask_module.py` | 디버그용 MJPEG 스트리밍 서버(RGB / 전처리 / ROI+오버레이 3분할 뷰). 현재 `final_main.py`에서는 직접 연결되어 있지 않은 보조 모듈 |
| `best.pt` | YOLO 가중치 파일 — 저장소에는 포함되어 있지 않으므로 `MODEL_PATH`가 가리키는 위치에 별도로 배치해야 함 |

`.env` / `.env.example`은 이 폴더가 아니라 **저장소 루트**(`MJY_Autonomous_Driving/.env`)에 있다.
`tx.py`, `final_main.py`는 `load_dotenv()`를 호출할 때 실행 파일 위치부터 상위 폴더로 올라가며 `.env`를 찾으므로,
프로젝트 폴더 안에 없어도 루트의 `.env`를 정상적으로 읽는다. (git에는 `.env.example`만 커밋됨)

## 환경 변수 (`.env`, 저장소 루트)

루트의 `.env.example`을 복사해 `.env`를 만들고 실제 IP/경로로 채운다. 아래 값 모두 필수이며, 없으면 즉시 에러가 난다.

| 변수 | 사용처 | 의미 |
|---|---|---|
| `FRAME_LISTEN_HOST` | tx.py | 노트북이 프레임 수신 대기할 주소 (보통 `0.0.0.0`) |
| `FRAME_LISTEN_PORT` | tx.py | 위 포트 |
| `PI_RESULT_HOST` | tx.py | 라즈베리파이의 결과 수신 서버 IP |
| `PI_RESULT_PORT` | tx.py | 위 포트 |
| `LAPTOP_IP` | final_main.py | 노트북(tx.py)의 IP |
| `LAPTOP_PORT` | final_main.py | 노트북의 프레임 수신 포트 (`FRAME_LISTEN_PORT`와 동일해야 함) |
| `PI_LISTEN_HOST` | final_main.py | 라즈베리파이가 결과 수신 대기할 주소 (보통 `0.0.0.0`) |
| `PI_LISTEN_PORT` | final_main.py | 위 포트 (`PI_RESULT_PORT`와 동일해야 함) |
| `MODEL_PATH` | tx.py | YOLO 가중치(`best.pt`) 경로 |

## 실행 방법

1. 저장소 루트에 `.env` 준비 (`.env.example` 참고, 실제 IP/포트/모델 경로로 채움)
2. 의존성 설치
   - 공통: `opencv-python`, `numpy`, `python-dotenv`
   - 노트북: `ultralytics` (YOLO)
   - 라즈베리파이: `afb` (보드 전용 GPIO/카메라 라이브러리), 필요 시 `flask`
3. **라즈베리파이에서 먼저** `final_main.py` 실행 → 결과 수신 서버가 대기 상태로 들어감
4. **노트북에서** `tx.py` 실행 → 라즈베리파이 결과 서버로 접속 후, 프레임 수신 서버를 열고 대기
   (`final_main.py`의 프레임 송신 스레드는 연결 실패 시 2초 간격으로 재시도하므로 순서가 다소 어긋나도 자동 재연결됨)
5. 라즈베리파이 콘솔에서 `s` 입력 시 주행 시작 트리거 발동 (`LaneFSM` state 1 → 2)

## LaneFSM 상태 요약

| state | 설명 |
|---|---|
| 1 | 정지, `s` 입력 대기 |
| 2 | 각도 기반 주행, `smallhill` 감지 시 3으로 |
| 3 | 언덕 부스트 주행(2.5초 가속 후 일반 주행), `rightsign` 감지 시 4로 |
| 4 | 5초간 우회전, 이후 `gateclosed` 감지 시 5로 |
| 5 | 정지 대기, `gateopen` 감지 시 6으로 |
| 6 | 주행, `staticb` 감지 시 7로 |
| 7 | 3초간 좌회전, 이후 `staticy` 감지 시 8로 |
| 8 | 3초간 우회전, 이후 `leftlight` 감지 시 9로 |
| 9 | 정지 대기, `greensign` 감지 또는 8초 경과 시 10으로 |
| 10 | 5초간 좌회전 후 일반 주행 복귀 |

## 주의사항

- `.env`는 저장소 루트에 있고 `.gitignore`에 등록되어 git에 커밋되지 않는다. 실제 IP·모델 경로는 `.env`에서만 관리한다.
- `MODEL_PATH`는 기본적으로 `best.pt`(상대 경로)로 설정하며, 이 경우 가중치 파일은 `tx.py` 실행 위치 기준으로 찾아지므로 `project(최종주행코드)/best.pt`에 둔다. 다른 위치에 두려면 `.env`의 `MODEL_PATH`를 절대 경로로 바꾸면 된다.
