# GestureDrive: remote gesture-controlled car

```
Laptop 1 (webcam + YOLO + Mosquitto) --Tailscale--> Laptop 2 (bridge) --Bluetooth--> ESP32-WROOM + L298N
```

| File | Runs on | Job |
|---|---|---|
| `inference.py` | Laptop 1 | Detects gestures and publishes `FORWARD/BACKWARD/LEFT/RIGHT/STOP` to `car_controls/` 5x per second |
| `bridge/mqtt_bt_bridge.py` | Laptop 2 | Subscribes over Tailscale and forwards each command to the ESP32 over Bluetooth |
| `esp32_car/` | ESP32 | ESP-IDF firmware: Bluetooth SPP server and L298N motor control |

## Safety: the car stops when
- the hand is a fist, nobody is in frame, or the gesture is uncertain (laptop 1)
- laptop 1 crashes or disconnects (MQTT Last Will publishes `STOP`)
- laptop 2 stops getting messages for 1 s, or loses the broker (bridge watchdog)
- the ESP32 gets no command for 1 s, or Bluetooth disconnects (firmware watchdog)

## 1. Laptop 1: Mosquitto must accept connections from the tailnet
Mosquitto 2.x only listens on localhost by default. Add this to `mosquitto.conf`
(Windows: `C:\Program Files\mosquitto\mosquitto.conf`), then restart the service:
```
listener 1883 0.0.0.0
allow_anonymous true
```
Then allow inbound TCP 1883 in Windows Firewall. Check from laptop 2:
`mosquitto_sub -h <laptop-1-tailscale-ip> -t car_controls/ -v`

Anyone on your tailnet can now publish to the car. That's fine for testing; add a password file later.

## 2. ESP32 firmware (ESP-IDF; builds clean on v6.0)
1. Check the pin `#define`s at the top of `esp32_car/main/main.c` against your wiring.
   Remove the ENA/ENB jumpers on the L298N so the ESP32's PWM controls speed.
   Connect the L298N GND to the ESP32 GND.
2. Build and flash:
   ```
   cd esp32_car
   idf.py set-target esp32
   idf.py build flash monitor
   ```
3. The monitor prints `Bluetooth MAC: ...` and `RFCOMM channel N`. Copy both into the bridge settings.

If a motor spins the wrong way, set `INVERT_LEFT` / `INVERT_RIGHT` to 1.
If the car doesn't move at low speed, raise `DRIVE_DUTY` / `TURN_DUTY` (max 1023).

## 3. Laptop 2: bridge
1. Pair "GestureDrive-Car" once in the OS Bluetooth settings (PIN `1234` if asked).
2. In `bridge/mqtt_bt_bridge.py`, set `BROKER` (laptop 1's Tailscale IP), `ESP32_MAC` and `RFCOMM_CHANNEL`.
3. `pip install paho-mqtt`, then `python mqtt_bt_bridge.py`

## 4. Laptop 1: run it
`pip install ultralytics paho-mqtt opencv-python`, then `python inference.py` (press `q` to quit; the car stops).

Start order doesn't matter: every part retries until the others are up.
