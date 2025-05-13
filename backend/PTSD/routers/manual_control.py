# routers/manual_control.py

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import paho.mqtt.publish as publish

router = APIRouter()

# MQTT 브로커 정보
MQTT_BROKER = "k12d101.p.ssafy.io"  # MQTT 브로커 주소
MQTT_TOPIC = "manual/control"  # MQTT 토픽

@router.websocket("/ws/manual-control")
async def control_robot(websocket: WebSocket):
    # WebSocket 연결 수락
    await websocket.accept()

    try:
        while True:
            # 클라이언트로부터 JSON 메시지 수신
            data = await websocket.receive_json()

            msg_type = data.get("type")  # 메시지 타입 (start 또는 stop)
            direction = data.get("direction")  # 방향 (W, A, S, D 등)
    
            # 메시지 타입에 따라 처리
            if msg_type == "start":
                # 로봇이 이동을 시작하는 경우
                print(f"[START] Direction: {direction}")
                # MQTT를 통해 로봇에 명령을 전송
                publish.single(MQTT_TOPIC, payload=direction, hostname=MQTT_BROKER)

            elif msg_type == "stop":
                # 로봇이 이동을 멈추는 경우
                print(f"[STOP] Direction: {direction}")
                # MQTT를 통해 로봇에 멈춤 명령을 전송
                publish.single(MQTT_TOPIC, payload="S", hostname=MQTT_BROKER)

    except WebSocketDisconnect:
        # WebSocket 연결이 끊어졌을 때 처리
        print("클라이언트와의 연결이 끊어졌습니다.")