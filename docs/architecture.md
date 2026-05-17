# PX4-XPlane Architecture & Data Flow

This document explains how the PX4 SITL process and the px4xplane X-Plane plugin
communicate, and how data moves through the system during simulation.

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          HOST MACHINE                                   │
│                                                                         │
│   ┌──────────────────────────┐        ┌──────────────────────────────┐  │
│   │     X-Plane Simulator    │        │      PX4 SITL Process        │  │
│   │                          │        │                              │  │
│   │  ┌────────────────────┐  │        │  ┌────────────────────────┐  │  │
│   │  │  px4xplane Plugin  │  │        │  │   Flight Controller    │  │  │
│   │  │  (.xpl binary)     │◄─┼──UDP──►│  │   (EKF2, Controllers, │  │  │
│   │  └────────────────────┘  │  4560  │  │    Mixer, Navigator)  │  │  │
│   │                          │        │  └────────────────────────┘  │  │
│   │  Flight Dynamics Engine  │        │                              │  │
│   │  Aircraft Model          │        │  Built with:                 │  │
│   │  Physics Simulation      │        │  make px4_sitl xplane_vtail  │  │
│   └──────────────────────────┘        └──────────────────────────────┘  │
│                                                                         │
│   ┌──────────────────────────────────────────────────────────────────┐  │
│   │                    QGroundControl (GCS)                          │  │
│   │                    Connects via UDP 14550                        │  │
│   └──────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

The plugin and PX4 can run on the same machine (localhost) or across a network
(e.g. X-Plane on Windows, PX4 SITL on WSL/Linux). Set `PX4_SIM_HOSTNAME` to
the remote IP when running across machines.

---

## 2. MAVLink Message Flow

```
X-Plane Flight Loop (each frame)
        │
        ▼
┌───────────────────┐     Sensor data (IMU, GPS,      ┌─────────────────────┐
│  DataRefManager   │────► Baro, Mag, Airspeed)        │                     │
│                   │     packed into MAVLink ─────────►│   PX4 SITL          │
│  Reads X-Plane    │                                  │                     │
│  datarefs:        │  HIL_SENSOR      (200 Hz)        │  EKF2               │
│  - position       │  HIL_GPS         ( 20 Hz)        │  ├─ State estimate  │
│  - attitude       │  HIL_STATE_QUAT  ( 10 Hz)        │  └─ Publishes:      │
│  - accel/gyro     │  HIL_RC_INPUTS   ( 10 Hz)        │     - attitude      │
│  - airspeed       │                                  │     - position      │
│  - baro           │◄─────────────────────────────────│     - velocity      │
│  - GPS            │  HIL_ACTUATOR_CONTROLS           │                     │
└───────────────────┘  (motor + servo commands)        │  Flight Controllers  │
        │                                              │  ├─ Rate/Attitude   │
        ▼                                              │  ├─ Position        │
┌───────────────────┐                                  │  └─ Navigation      │
│  DataRefManager   │                                  └─────────────────────┘
│  writeActuators() │
│                   │
│  Maps ch0..chN    │
│  → X-Plane        │
│    datarefs       │
│  (motors, servos) │
└───────────────────┘
```

---

## 3. Plugin Internal Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    px4xplane Plugin                         │
│                                                             │
│  X-Plane Entry Points                                       │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  XPluginStart()  → load config, open UDP socket     │   │
│  │  XPluginStop()   → close socket, cleanup            │   │
│  │  FlightLoopCallback() → runs every X-Plane frame    │   │
│  └──────────────────────────┬──────────────────────────┘   │
│                             │                               │
│            ┌────────────────┼──────────────────┐           │
│            ▼                ▼                  ▼           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ ConfigManager│  │ConnectionMgr │  │  MAVLinkManager  │  │
│  │              │  │              │  │                  │  │
│  │ Loads and    │  │ UDP socket   │  │ sendHILSensor()  │  │
│  │ parses       │  │ to PX4 SITL  │  │ sendHILGPS()     │  │
│  │ config.ini   │  │ (port 4560)  │  │ sendHILState()   │  │
│  │              │  │              │  │ receiveActuators()│  │
│  │ channel0..N  │  │ recv/send    │  │                  │  │
│  │ → datarefs   │  │ raw bytes    │  │ Builds MAVLink   │  │
│  └──────────────┘  └──────────────┘  │ packets from     │  │
│            │                         │ DataRefManager   │  │
│            ▼                         └────────┬─────────┘  │
│  ┌──────────────┐                            │             │
│  │ DataRefManager                            ▼             │
│  │              │                  ┌──────────────────┐    │
│  │ Reads sensor │                  │  ActuatorSafety  │    │
│  │ datarefs     │                  │                  │    │
│  │ from X-Plane │                  │ Clamps outputs   │    │
│  │              │                  │ Detects stale    │    │
│  │ Writes       │                  │ commands → zero  │    │
│  │ actuator     │                  └──────────────────┘    │
│  │ datarefs     │                                          │
│  └──────────────┘                                          │
│                                                             │
│  Support Modules                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │AccelCalibra- │  │ TimeManager  │  │ConnectionStatusHUD│  │
│  │tion          │  │Timestamp-    │  │FPSMonitor        │  │
│  │Corrects g    │  │Provider      │  │UIHandler         │  │
│  │offset on     │  │High-precision│  │                  │  │
│  │startup       │  │timestamps    │  │In-sim overlay    │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Sensor Data Pipeline (X-Plane → PX4)

```
X-Plane DataRefs
      │
      │  sim/flightmodel/forces/g_axil  (body accel X)
      │  sim/flightmodel/forces/g_side  (body accel Y)
      │  sim/flightmodel/forces/g_nrml  (body accel Z)
      │  sim/flightmodel/position/P/Q/R (body rates)
      │  sim/flightmodel/position/theta/phi/psi
      │  sim/flightmodel/position/q     (quaternion)
      │  sim/flightmodel/position/elevation
      │  sim/flightmodel/position/latitude/longitude
      │  sim/flightmodel/position/indicated_airspeed
      │
      ▼
┌─────────────────────────────────┐
│  AccelCalibration               │
│  (corrects g_nrml offset)       │
│                                 │
│  TimestampProvider              │
│  (monotonic μs timestamps)      │
└────────────────┬────────────────┘
                 │
                 ▼
┌─────────────────────────────────┐
│  MAVLinkManager                 │
│                                 │
│  HIL_SENSOR message:            │
│  ├─ xacc, yacc, zacc  (m/s²)   │
│  ├─ xgyro,ygyro,zgyro (rad/s)  │
│  ├─ xmag, ymag, zmag  (gauss)  │
│  ├─ abs_pressure      (hPa)    │
│  ├─ pressure_alt      (m)      │
│  ├─ temperature       (°C)     │
│  └─ time_usec                  │
│                                 │
│  HIL_GPS message:               │
│  ├─ lat, lon, alt               │
│  ├─ vel, vn, ve, vd             │
│  ├─ cog, eph, epv               │
│  └─ satellites_visible          │
└────────────────┬────────────────┘
                 │ UDP 4560
                 ▼
           PX4 SITL EKF2
```

---

## 5. Actuator Command Pipeline (PX4 → X-Plane)

```
PX4 SITL
  │
  │  HIL_ACTUATOR_CONTROLS message
  │  controls[0..15] normalized [-1, +1]
  │
  ▼
┌──────────────────────────────────────┐
│  ConnectionManager::receiveData()    │
│  Parses raw MAVLink bytes from UDP   │
└─────────────────┬────────────────────┘
                  │
                  ▼
┌──────────────────────────────────────┐
│  ActuatorSafety                      │
│  ├─ Checks timestamp freshness       │
│  ├─ If stale > 500ms → zero outputs  │
│  └─ Clamps to finite values          │
└─────────────────┬────────────────────┘
                  │
                  ▼
┌──────────────────────────────────────┐
│  DataRefManager::writeActuators()    │
│                                      │
│  Reads channel mapping from          │
│  ConfigManager (config.ini):         │
│                                      │
│  channel0 → dataref, range [min max] │
│  channel1 → dataref, range [min max] │
│  ...                                 │
│                                      │
│  Normalizes [-1,+1] → [min, max]     │
│  Writes to X-Plane dataref           │
└─────────────────┬────────────────────┘
                  │
                  ▼
         X-Plane DataRefs
         sim/flightmodel/engine/ENGN_thro_use[n]
         sim/flightmodel/controls/wing*_ail*def
         sim/flightmodel/controls/hstab*_elv*def
         sim/flightmodel/controls/vstab*_rud*def
         sim/flightmodel2/engines/nacelle_angle_deg[n]
```

---

## 6. Airframe Configuration Flow

```
PX4-Autopilot-Me repo
  │
  ├─ ROMFS/px4fmu_common/init.d-posix/airframes/
  │    5001_xplane_cessna172
  │    5002_xplane_tb2
  │    5003_xplane_vtail          ← sets CA params, PWM_MAIN_FUNC mapping
  │    5010_xplane_ehang184
  │    5020_xplane_alia250
  │    5021_xplane_qtailsitter
  │
  ├─ src/modules/simulation/simulator_mavlink/
  │    sitl_targets_xplane.cmake  ← registers make targets
  │    add_xplane_target(xplane_vtail 5003 5003_xplane_vtail)
  │
  └─ build command:
       make px4_sitl_default xplane_vtail
            │
            └─ launches PX4 with SYS_AUTOSTART=5003
               PX4 reads airframe script → sets CA/PWM params
               PX4 waits for HIL connection on UDP 4560


px4xplane repo
  │
  └─ config/config.ini
       config_name = VTail      ← selects active section
       │
       [VTail]
       channel0 = <dataref>, <type>, <index>, [min max]
       channel1 = ...           ← maps PX4 PWM_MAIN_FUNC order
       channel2 = ...              to X-Plane datarefs
       ...


Mapping contract between the two sides
  PX4 PWM_MAIN_FUNC1 → HIL_ACTUATOR_CONTROLS controls[0] → config.ini channel0
  PX4 PWM_MAIN_FUNC2 → HIL_ACTUATOR_CONTROLS controls[1] → config.ini channel1
  ...
```

---

## 7. Startup Sequence

```
1. Launch X-Plane
      │
      └─► px4xplane plugin loads
            ├─ reads config.ini  (selects airframe section)
            ├─ opens UDP socket on port 4560 (listen)
            └─ starts FlightLoopCallback

2. Run: make px4_sitl_default xplane_vtail
      │
      └─► PX4 starts
            ├─ sets SYS_AUTOSTART=5003
            ├─ loads airframe params (CA, PWM_MAIN_FUNC, EKF2)
            ├─ enters HIL mode (waits for HIL_SENSOR)
            └─ connects to X-Plane on UDP 4560

3. First HIL_SENSOR received by PX4
      │
      └─► EKF2 initializes
            ├─ aligns IMU
            ├─ acquires GPS fix
            └─ state estimate becomes valid

4. Open QGroundControl
      └─► connects via UDP 14550 (auto-detected)
            ├─ shows vehicle on map
            └─ ready to arm and fly
```

---

## 8. Network Ports Reference

| Port  | Protocol | Direction              | Purpose                         |
|-------|----------|------------------------|---------------------------------|
| 4560  | UDP      | Plugin → PX4           | HIL sensor data                 |
| 4560  | UDP      | PX4 → Plugin           | HIL actuator commands           |
| 14550 | UDP      | PX4 → QGroundControl   | MAVLink telemetry (GCS link)    |
| 14540 | UDP      | PX4 → SDK/MAVSDK       | Offboard API                    |

For cross-machine setup (X-Plane on Windows, PX4 on WSL):
```bash
export PX4_SIM_HOSTNAME=<windows-host-ip>
make px4_sitl_default xplane_vtail
```

---

## 9. Config.ini Channel Mapping Reference

```
config.ini [section]          PX4 airframe file
─────────────────             ──────────────────
channel0  ←──────────────────  PWM_MAIN_FUNC1  (CA_SV_CS0 or motor)
channel1  ←──────────────────  PWM_MAIN_FUNC2
channel2  ←──────────────────  PWM_MAIN_FUNC3
channel3  ←──────────────────  PWM_MAIN_FUNC4
channel4  ←──────────────────  PWM_MAIN_FUNC5
...

Channel value format:
  channel0 = <dataref_path>, <type>, <array_index>, [min max]

  dataref_path  : X-Plane dataref string
  type          : float | floatArray
  array_index   : 0 for float, [N] for floatArray
  min max       : output range in X-Plane units (degrees, throttle 0-1, etc.)

Multiple datarefs on one channel (pipe-separated):
  channel0 = sim/.../wing1l_ail1def, float, 0, [-20 20] | sim/.../wing2l_ail1def, float, 0, [-20 20]
```
