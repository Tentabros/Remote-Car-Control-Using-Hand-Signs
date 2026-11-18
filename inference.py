from collections import Counter, deque
import time
import uuid

import cv2
from ultralytics import YOLO
from paho.mqtt import client as mqtt_client

# ---------------- comp vis settings ----------------
MODEL_PATH = "best.pt"
CAMERA_INDEX = 0
IMGSZ = 512
CONF_THRESHOLD = 0.75
VOTE_WINDOW = 7
VOTE_RATIO = 0.70

COMMANDS = {
    "like": "FORWARD",
    "dislike": "BACKWARD",
    "peace": "LEFT",
    "rock": "RIGHT",
    "fist": "STOP",
    "no_gesture": "STOP",
}

# ---------------- mqtt settings ----------------
# USE_CLOUD = True  -> HiveMQ Cloud (works from anywhere: PC at home, car on your phone's hotspot)
# USE_CLOUD = False -> your own Mosquitto (same network, or a Tailscale IP)
USE_CLOUD = False

CLOUD_HOST = "YOUR-CLUSTER-ID.s1.eu.hivemq.cloud"  # from the HiveMQ Cloud console
CLOUD_PORT = 8883                                  # TLS only
MQTT_USER = "YOUR_USERNAME"                        # HiveMQ "Access Management" credentials
MQTT_PASS = "YOUR_PASSWORD"                        # don't push real credentials to GitHub

LOCAL_HOST = "localhost"                            # Mosquitto runs on this laptop (laptop 1)
LOCAL_PORT = 1883

BROKER = CLOUD_HOST if USE_CLOUD else LOCAL_HOST
PORT = CLOUD_PORT if USE_CLOUD else LOCAL_PORT
TOPIC = "car_controls/"            # the ESP32 must subscribe to exactly this topic
CLIENT_ID = f"python-mqtt-{uuid.uuid4().hex[:6]}"  # unique, so two clients never kick each other off
HEARTBEAT_S = 0.2                  # re-send the current command 5x per second (for the ESP32 watchdog)


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"Connected to MQTT broker {BROKER}:{PORT}")
    else:
        print(f"MQTT connection failed: {reason_code}")


def on_disconnect(client, userdata, flags, reason_code, properties):
    if reason_code != 0:  # unexpected drop; loop_start() reconnects automatically
        print(f"MQTT disconnected ({reason_code}), reconnecting...")


def connect_mqtt():
    client = mqtt_client.Client(
        client_id=CLIENT_ID,
        callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
    )
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    if USE_CLOUD:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.tls_set()  # TLS with the system's trusted CAs (HiveMQ uses Let's Encrypt)
    # Last Will: if this script crashes or loses connection, the broker publishes STOP for us
    client.will_set(TOPIC, "STOP", qos=1)
    client.connect(BROKER, PORT, keepalive=10)
    client.loop_start()   # background thread: sends/receives packets, keepalives, auto-reconnect
    return client


def send_command(client, cmd, log=True):
    result = client.publish(TOPIC, cmd)
    if log:
        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f">>> command | {cmd} | sent")
        else:
            print(f">>> failed to send {cmd} (rc={result.rc})")
    return result


# ---------------- setup ----------------
client = connect_mqtt()          # connect FIRST, before anything is published

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

history = deque(maxlen=VOTE_WINDOW)
current_cmd = "STOP"
send_command(client, current_cmd)
prev_time = time.time()
last_sent = time.time()

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        frame = cv2.flip(frame, 1)  # mirror view, like a selfie camera

        results = model(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD, verbose=False)
        boxes = results[0].boxes

        # Use only the most confident hand in the frame (the "driver" hand)
        if len(boxes) > 0:
            best = int(boxes.conf.argmax())
            gesture = model.names[int(boxes.cls[best])]
            confidence = float(boxes.conf[best])
            frame_cmd = COMMANDS.get(gesture, "STOP")
        else:
            gesture, confidence, frame_cmd = "none", 0.0, "STOP"  # no hand = STOP

        # Temporal smoothing: only switch when most of the recent frames agree
        history.append(frame_cmd)
        top_cmd, votes = Counter(history).most_common(1)[0]
        now = time.time()
        if len(history) == VOTE_WINDOW and votes / VOTE_WINDOW >= VOTE_RATIO and top_cmd != current_cmd:
            current_cmd = top_cmd
            send_command(client, current_cmd)          # new command: send immediately and log it
            last_sent = now
        elif now - last_sent >= HEARTBEAT_S:
            send_command(client, current_cmd, log=False)  # same command: quiet heartbeat
            last_sent = now

        # ---------------- display ----------------
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        annotated = results[0].plot()
        color = (0, 0, 255) if current_cmd == "STOP" else (0, 200, 0)
        cv2.putText(annotated, f"CMD: {current_cmd}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.putText(annotated, f"{gesture} {confidence:.2f} | {fps:.0f} FPS", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Car Controller", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    # Always stop the car on exit, even after an error or Ctrl+C
    try:
        send_command(client, "STOP").wait_for_publish(timeout=2)
    except Exception as e:
        print("Could not send final STOP:", e)
    client.loop_stop()
    client.disconnect()
    cap.release()
    cv2.destroyAllWindows()
