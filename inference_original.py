from collections import Counter, deque
import time

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
broker = "broker.mosquitto.io"
port = 1883
topic = "car_controls/"
client_id = "python-mqtt-1"

def connect_mqtt():

    client = mqtt_client.Client(
        client_id = client_id,
        callback_api_version= mqtt_client.CallbackAPIVersion.VERSION2,
    )

    client.connect(broker, port)

    return client

def send_command(cmd):
    """Called only when the command changes. Replace the print with your MQTT publish later."""
    print(f">>> {cmd}")
    time.sleep(1)
    result = client.publish(topic,cmd)

    status = result[0]

    if status == 0:
        print(f">>> command | {cmd} | sent")

    else:
        print("failed to send message")

        


# ---------------- setup ----------------
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

history = deque(maxlen=VOTE_WINDOW)
current_cmd = "STOP"
send_command(current_cmd)
prev_time = time.time()

connect_mqtt()

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)  # mirror view, like a selfie camera

    # conf filters weak boxes; verbose=False stops the per-frame console spam (which slows the loop)
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
    if len(history) == VOTE_WINDOW and votes / VOTE_WINDOW >= VOTE_RATIO and top_cmd != current_cmd:
        current_cmd = top_cmd
        send_command(current_cmd)

    # ---------------- display ----------------
    now = time.time()
    fps = 1.0 / max(now - prev_time, 1e-6)
    prev_time = now

    annotated = results[0].plot()
    color = (0, 0, 255) if current_cmd == "STOP" else (0, 200, 0)
    cv2.putText(annotated, f"CMD: {current_cmd}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    cv2.putText(annotated, f"{gesture} {confidence:.2f} | {fps:.0f} FPS", (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imshow("Car Controller", annotated)

    send_command(current_cmd)

    # waitKey must run EVERY frame: it is what lets the window redraw
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

send_command("STOP")
cap.release()
cv2.destroyAllWindows()