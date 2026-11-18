"""
Laptop 2: MQTT (over Tailscale) -> Bluetooth Classic -> ESP32 car

Subscribes to the car commands published by laptop 1 and forwards each one to the ESP32
as a line of text ("FORWARD\n"). Works on Windows and Linux with the standard library's
Bluetooth sockets. The only package needed is paho-mqtt.

    pip install paho-mqtt
    python mqtt_bt_bridge.py
"""
import socket
import threading
import time
import uuid

from paho.mqtt import client as mqtt_client

# ---------------- settings ----------------
BROKER = "100.110.103.115"      # laptop 1's Tailscale IP (where Mosquitto runs)
PORT = 1883
TOPIC = "car_controls/"         # same topic inference.py publishes to

ESP32_MAC = "XX:XX:XX:XX:XX:XX" # printed by the ESP32 on boot ("Bluetooth MAC: ...")
RFCOMM_CHANNEL = 1              # printed by the ESP32 on boot ("RFCOMM channel N")

VALID = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}
STALE_S = 1.0                   # no MQTT message for this long -> send STOP

# ---------------- bluetooth ----------------
bt_sock = None
bt_lock = threading.Lock()


def bt_connect():
    """Keep trying until the ESP32 accepts the connection."""
    global bt_sock
    while True:
        try:
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.connect((ESP32_MAC, RFCOMM_CHANNEL))
            with bt_lock:
                bt_sock = s
            print(f"[BT] connected to {ESP32_MAC} ch {RFCOMM_CHANNEL}")
            return
        except OSError as e:
            print(f"[BT] connect failed ({e}), retrying in 2 s...")
            time.sleep(2)


def bt_send(cmd):
    """Send one command line. On failure, drop the socket so the watchdog thread reconnects."""
    global bt_sock
    with bt_lock:
        if bt_sock is None:
            return False
        try:
            bt_sock.sendall((cmd + "\n").encode())
            return True
        except OSError as e:
            print(f"[BT] send failed ({e}), will reconnect")
            try:
                bt_sock.close()
            except OSError:
                pass
            bt_sock = None
            return False


# ---------------- mqtt ----------------
last_msg_time = 0.0
last_cmd = None


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"[MQTT] connected to {BROKER}:{PORT}, subscribing to '{TOPIC}'")
        client.subscribe(TOPIC, qos=0)  # (re)subscribe on every (re)connect
    else:
        print(f"[MQTT] connection failed: {reason_code}")


def on_disconnect(client, userdata, flags, reason_code, properties):
    print(f"[MQTT] disconnected ({reason_code}) -> STOP, reconnecting...")
    bt_send("STOP")


def on_message(client, userdata, msg):
    global last_msg_time, last_cmd
    cmd = msg.payload.decode(errors="ignore").strip().upper()
    if cmd not in VALID:
        print(f"[MQTT] ignoring unknown payload {msg.payload!r}")
        return
    last_msg_time = time.time()
    if bt_send(cmd) and cmd != last_cmd:  # forward every message (keeps the ESP32 watchdog fed)
        print(f">>> {cmd}")
        last_cmd = cmd


def watchdog():
    """Reconnect Bluetooth when needed and send STOP if laptop 1 goes quiet."""
    global last_cmd
    stopped_for_stale = False
    while True:
        if bt_sock is None:
            bt_connect()
            last_cmd = None
        stale = time.time() - last_msg_time > STALE_S
        if stale and not stopped_for_stale:
            bt_send("STOP")
            if last_cmd not in (None, "STOP"):
                print("[WATCHDOG] no commands from laptop 1 -> STOP")
            last_cmd = "STOP"
            stopped_for_stale = True
        elif not stale:
            stopped_for_stale = False
        time.sleep(0.1)


def main():
    bt_connect()
    threading.Thread(target=watchdog, daemon=True).start()

    client = mqtt_client.Client(
        client_id=f"car-bridge-{uuid.uuid4().hex[:6]}",
        callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
    )
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)

    while True:  # keep trying until laptop 1's broker is reachable
        try:
            client.connect(BROKER, PORT, keepalive=10)
            break
        except OSError as e:
            print(f"[MQTT] cannot reach {BROKER}:{PORT} ({e}), retrying in 2 s...")
            time.sleep(2)

    try:
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping car and exiting")
        bt_send("STOP")
        client.disconnect()
        with bt_lock:
            if bt_sock is not None:
                bt_sock.close()


if __name__ == "__main__":
    main()
