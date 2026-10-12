전체 공정 웹캠 송출 패키지 - CMD 깨짐 수정본

1. 최초 한 번 0_install_requirements.bat 실행
2. 웹캠 연결
3. 1_start_full_process_camera.bat 실행
4. 브라우저에서 http://127.0.0.1:5100/video 확인
5. 통합 GUI 실행

GUI 설정:
DEFAULT_FULL_HOST = "127.0.0.1"
DEFAULT_FULL_PORT = 5100

카메라가 잘못 선택되면 full_process_camera_stream.py 위쪽에서:
AUTO_FIND_CAMERA = False
CAMERA_INDEX = 1
