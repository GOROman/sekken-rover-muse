# Sekken Rover Muse （せっけんロボ ミューズ）

A personal AI agent that can move around the room: a two-wheel self-balancing
robot built on the **D-Robotics RDK X5**, connected to the author's Muse
Gadgets ecosystem so the agent can act physically, not only in chat.

Project for the **RDK Challenge 2026 Winter** (Fun Robotics track).
Participant: Yoshihito Kondoh (XVI Inc.)

- Drive: **two-wheel inverted pendulum**, IMU-based attitude control
- Behavior: on low battery the robot **returns to its charger by itself**
- Brain: D-Robotics RDK X5 (owned by the participant)
- Agent link: Muse Gadgets — the same agent that manages the author's
  mail, calendar and smart home also rides this robot

> **Status (2026-11-06): Stage 1 in progress.**
> The control skeleton below is verified in simulation (12 unit tests).
> Hardware bring-up on the real board (flash, SSH, sensor, on-device AI) is
> scheduled for the Nov 9–10 session, and the evidence links in this README
> will be filled in from that session. Nothing in this repository claims
> real-hardware results that have not been measured yet.
> The IMU (M5Stack IMU Unit, I2C) is in transit, expected around Nov 10;
> the Stage 1 sensor/AI evidence therefore starts with the camera path,
> which works without the IMU.

## Hardware

| Part | Notes |
|---|---|
| D-Robotics RDK X5 | Main board, owned |
| USB camera | Person detection / confirmation photos (Stage 1 sensor + AI) |
| M5Stack IMU Unit (I2C) | Attitude sensing for the inverted pendulum; in transit, ETA ~Nov 10 |
| 2-wheel chassis + encoder motors | Stage 2 (balancing on real hardware) |
| Motor driver, battery, charging dock | Stage 2 |

## Stage 1 Bring-Up (RDK X5)

Procedure used for the Stage 1 evidence. Credentials, Wi-Fi passwords and
device serials are never published in this repository.

### 1. Flash the system image

1. Install **RDK Studio** on the development PC (official D-Robotics tool).
2. Download the latest officially supported RDK X5 system image.
3. Flash the image to the board with RDK Studio and let the board boot.

### 2. Connect and log in over SSH

1. Connect the board by Ethernet (or join it to the local Wi-Fi).
2. Find the board's IP address on the router / via the serial console.
3. Log in from the development PC using the credentials in the official
   RDK X5 documentation, then change the password immediately:

```bash
ssh root@<RDK_X5_IP>
```

4. Sanity checks once logged in:

```bash
uname -a
cat /etc/os-release
ip addr          # confirm network address
ping -c 3 8.8.8.8  # confirm internet reachability
```

### 3. Sensor check

USB camera:

```bash
ls /dev/video*
```

IMU over I2C (after the M5Stack IMU Unit arrives):

```bash
i2cdetect -y <bus>   # confirm the IMU answers on its I2C address
```

### 4. First AI task (on-device)

Person/object detection running on the board's BPU, starting from the
official demo zoo:

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
# then follow the YOLO demo README for the RDK X5 inside the cloned repo
```

The submitted AI evidence is a screenshot of the detection demo running on
this participant's own board.

## Control Skeleton (simulation stage)

「せっけんロボ ミューズ」のソフトウェア骨格。現段階の検証はすべて
シミュレーション上で行っている。

| モジュール | 内容 |
|---|---|
| `sekken_rover/plant.py` | 2輪倒立振子の物理モデル（検証用プラント） |
| `sekken_rover/imu.py` | IMU 接続点。仮想 IMU と実機用スタブ（`M5StackImuSource`） |
| `sekken_rover/controller.py` | 倒立振子 PID 制御（速度ループ＋姿勢ループ） |
| `sekken_rover/battery.py` | バッテリー監視と自動帰還の判断（低残量 20% / 危険 8%） |
| `sekken_rover/robot.py` | 状態機械: balancing → returning → charging、転倒保護 |
| `sekken_rover/sim.py` | シミュレーション実行（バランス／バッテリー帰還の2シナリオ） |

### Run it

```bash
# Tests (Python standard library only, nothing to install)
python3 -m unittest discover -s tests -v

# Recover from a 10-degree tilt + disturbance rejection
python3 -m sekken_rover.sim balance

# Low battery -> auto return -> docking -> charge complete
python3 -m sekken_rover.sim battery
```

### What the tests verify

- Corrective output points the right way for both forward and backward tilt
- Attitude converges from an initial 10-degree tilt; recovers from a push
- The robot tracks a target velocity
- Crossing the battery thresholds fires `battery_low` and walks the state
  machine through return → docking → charge complete
- No return is triggered while the battery is healthy
- The IMU is swappable, and the real-hardware stub is explicitly unimplemented

## Dependencies

| Component | Dependency |
|---|---|
| Control skeleton + tests | Python 3, standard library only |
| On-device AI demos | Official `rdk_model_zoo` / `rdk-course-demos` (D-Robotics), matched to the flashed system image |
| Flashing | RDK Studio (official) |

## Troubleshooting

| Symptom | Check |
|---|---|
| Board does not boot after flashing | Re-flash with RDK Studio; confirm the power supply meets the RDK X5 requirement and the image matches the board |
| SSH connection refused | Board and PC on the same subnet; board fully booted; try the serial console to read the boot log |
| Board has an IP but no internet | Gateway/DNS on the router; re-test with `ping` to the gateway first, then to a public IP |
| USB camera not listed | Re-seat the cable, check `ls /dev/video*` and `dmesg`; MIPI cameras use a different pipeline than USB UVC |
| Model zoo demo errors on the BPU | Demo and model versions must match the flashed system image; re-clone the zoo after an image upgrade |
| IMU not answering on I2C | Check wiring (SDA/SCL/GND/power) and scan the correct bus with `i2cdetect` |

## Stage 1 Evidence

Captured on the real board on Nov 7, 2026:

- [x] Flash + SSH evidence — RDK OS 3.5.0 Desktop on a 32GB microSD, first boot verified (rootfs auto-expanded); SSH over Ethernet (netplan reconfigured to DHCP; the wired factory default is a fixed 192.168.127.10) and Wi-Fi. Screenshots attached to the community post below
- [x] Sensor evidence — USB camera (Insta360 Ace Pro in UVC mode) live frames with YOLOv5x detection on the BPU; the camera was pointed at the RDK X5 board itself (no people filmed, for privacy)
- [x] AI evidence — YOLOv8x from the official pydev_demo on the BPU: 13 detections on the test image (7x person, 6x kite)
- [x] Community post (Discord, Stage 1 completion in the introduction thread): https://discord.com/channels/1300358874280230994/1568442571301068850/1568525991733887056
- [x] Showcase PR (challenge repository, `projects/`): https://github.com/D-Robotics/Robotics-Dream-Keeper-Challenge-Winter/pull/2

## After the IMU arrives

`docs/IMU_INTEGRATION.md` collects the integration points and tuning
checklist: implement `M5StackImuSource` (I2C, axis mapping, bias correction,
filtering), re-tune the PID gains on real hardware, calibrate the battery
thresholds against measurements, and verify the fall protection on the board.

## Scope

In scope for this skeleton: the two-wheel inverted pendulum and low-battery
auto-return only. Person following, obstacle avoidance, the Muse agent API
and SLAM are Stage 2+ work.

## Safety

- No Wi-Fi passwords, tokens, private keys or device serials are published.
- Real-hardware motion tests run at reduced speed with a clear fall zone;
  fall protection in the state machine cuts drive output on excessive tilt.

## License

MIT — see `LICENSE`.
