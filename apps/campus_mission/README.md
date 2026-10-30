# Campus Mission

기존 ROS 2 주행 위에서 실행하는 웹 관제·자연어 임무·현장 관찰 앱이다.
로컬 모델 대신 외부 LLM/VLM API를 사용하며 목적지는 지도별 YAML로 관리한다.

```text
campus_mission/  FastAPI, Mission Manager, ROS 연결, 시작 절차
web/src/        React 화면
web/dist/       Node.js 없이 배포할 수 있는 빌드 결과
web/package*    프런트엔드 의존성과 lockfile
config/         공개 가능한 초기 목적지 예제
deploy/         systemd 서비스와 설치·실행 스크립트
tests/          임무·HTTP·격리된 ROS 연결 테스트
pyproject.toml  Python 의존성
.env.example    비밀값 없는 배포 설정 예제
```

실제 비밀키·장비 설정은 `~/.config/campus-rover/mission.env`, 지도별 좌표는
`~/ros2_ws/maps/<지도>/places.yaml`, 임무 기록과 사진은 `~/ros2_ws/mission_data/`에 둔다.
`apps/COLCON_IGNORE`로 ROS 패키지 빌드와 분리한다.

실행과 지도 교체는 [웹 임무 안내](../../docs/campus_missions.md)를 따른다.
공통 진입 명령은 `bash ~/ros2_ws/src/tools/campus_web.sh`다.

화면 수정 후 Node.js 22 환경에서:

```bash
cd ~/ros2_ws/src/apps/campus_mission/web
npm ci
npm run build
```

소프트웨어 테스트에는 pytest와 FastAPI TestClient용 HTTP 클라이언트가 필요하다.
ROS 연결 테스트는 `ROS_DOMAIN_ID=91`의 합성 센서·가짜 MAVROS만 사용한다.
부팅 시 웹과 외부 HTTPS 연결을 자동 실행하며 주행 서비스·시동은 사용자가 웹에서 요청할 때 실행한다.
