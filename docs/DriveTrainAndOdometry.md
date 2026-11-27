# DriveTrain & Odometry Subsystem Documentation

This document explains the architecture, kinematics, control theory, and sensor fusion behind the **DriveTrain** and **Odometry** subsystems in BioBuzz.

---

## 1. High-Level Architecture Overview

The robot's mobility and localization pipeline consists of two main subsystems working in tandem:

* **DriveTrain Subsystem (`DriveTrain.java`)**: Handles robot motion, field-centric and robot-centric driving kinematics, and per-motor closed-loop PIF velocity control.
* **Odometry Subsystem (`Subsystems/Odometry/`)**: Tracks the robot's pose $(X, Y, \theta)$ on the field using hardware dead-wheel odometry pods fused with Limelight vision updates.

```
                           DRIVER / AUTO COMMANDS
                                     │
                      [FieldDrive / RobotDrive]
                                     │
                                     ▼
                      ┌─────────────────────────────┐
                      │    DriveTrain Subsystem     │
                      │  - Kinematics (Wheel Speeds)│
                      │  - PIF Motor Velocity Loop  │
                      └──────────────┬──────────────┘
                                     │ (Powers)
                                     ▼
                               Drive Motors
                                     │
                                     ▼ (Movement)
                              Robot on Field
                                     │
        ┌────────────────────────────┼───────────────────────────┐
        ▼                            ▼                           ▼
[Pinpoint Dead-Wheels]        [Control Hub Gyro]       [Limelight MegaTag2]
        │                            │                           │
        ▼                            ▼                           ▼
PinpointOdometry.java            Gyro.java                 LimeLight.java
        │                            │                           │
        └────────────────────┬───────┴───────────────────────────┘
                             ▼
                    ┌─────────────────────────────┐
                    │      Odometry.java          │
                    │  - Dead-wheel Pose Update   │
                    │  - Speed-gated Vision Blend │
                    │  - Cross-OpMode Persistence │
                    └─────────────────────────────┘
                             │
                             ▼
                  Current Field Pose (X, Y, θ)
```

---

## 2. DriveTrain Subsystem (`DriveTrain.java`)

`DriveTrain` extends FTCLib's `SubsystemBase` to control a 4-wheel mecanum drive chassis.

### 2.1 Hardware Configuration
* **Motors**: 4 `DcMotorEx` instances (`leftFrontDrive`, `leftBackDrive`, `rightFrontDrive`, `rightBackDrive`).
* **Directions**:
  * Left side motors are set to `FORWARD`.
  * Right side motors are set to `REVERSE`.
* **Motor RunMode**: Motors are configured to `RUN_WITHOUT_ENCODER`. Instead of relying on the SDK's built-in velocity PID, closed-loop speed control is handled directly in software by custom PIF controllers in `DriveTrain.periodic()`.
* **Zero Power Behavior**: Set to `BRAKE` on all four motors for snappy stops and predictable deceleration.

### 2.2 Physical Geometry & Kinematics Constants
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `MAXRPM` | $6000.0\text{ RPM}$ | Max unloaded motor speed |
| `TICKS_PER_ROTATION` | $28.0$ | Motor shaft encoder counts per revolution |
| `WHEEL_DIA` | $0.104\text{ m}$ ($104\text{ mm}$) | Mecanum wheel diameter |
| `GEAR_RATIO` | $20.0$ | Drive gear reduction ratio |
| `TRACK_WIDTH` | $0.34\text{ m}$ | Distance between left and right wheels |
| `TRACK_LENGTH` | $0.192\text{ m}$ | Distance between front and back axles |
| `MAX_SPEED` | $\approx 1.63\text{ m/s}$ | Maximum theoretical linear speed: $\frac{\text{MAXRPS} \times \pi \times \text{WHEEL\_DIA}}{\text{GEAR\_RATIO}}$ |

Kinematics geometry is instantiated using FTCLib's `MecanumDriveKinematics` with center-of-rotation offsets:
* **Front Left**: `(+0.208 * TRACK_LENGTH, +0.5 * TRACK_WIDTH)`
* **Front Right**: `(+0.208 * TRACK_LENGTH, -0.5 * TRACK_WIDTH)`
* **Back Left**: `(-0.792 * TRACK_LENGTH, +0.5 * TRACK_WIDTH)`
* **Back Right**: `(-0.792 * TRACK_LENGTH, -0.5 * TRACK_WIDTH)`

### 2.3 Driving Modes

#### Robot-Centric Driving (`RobotDrive`)
Given desired linear velocities ($V_x$, $V_y$ in $\text{m/s}$) and angular velocity ($\Omega$ in $\text{rad/s}$):
1. Pack velocities into a `ChassisSpeeds(Vx, Vy, Omega)` object.
2. Calculate target wheel speeds using `driveKinematics.toWheelSpeeds(driveChassisSpeeds, new Translation2d(0, 0))`.
3. Normalize speeds: If the magnitude of any wheel's requested speed exceeds `MAX_SPEED`, all wheel speeds are scaled down proportionally to preserve the intended motion vector.
4. Apply the resulting `ReferenceWheelSpeeds` to the motors.

#### Field-Centric Driving (`FieldDrive`)
Translates driver joystick commands (relative to the field perimeter) into robot-centric commands:
1. Queries the current heading angle $\theta$ (in radians) from `RobotContainer.gyro.getYawAngle()`.
2. Applies a 2D coordinate rotation matrix by negative gyro angle ($-\theta$):
   $$x = V_x \cos(-\theta) - V_y \sin(-\theta)$$
   $$y = V_x \sin(-\theta) + V_y \cos(-\theta)$$
3. Calls `RobotDrive(x, y, Omega)`.

### 2.4 Closed-Loop PIF Speed Control (`MotorControl`)
Each motor has a dedicated instance of the inner `MotorControl` class implementing a **Proportional-Integral-Feedforward (PIF)** control algorithm:

* **Controller Gains**:
  * $K_p = 1.25 \times 0.00016667 \approx 0.000208$
  * $K_i = 5.5 \times 0.00016667 \approx 0.000917$
  * $K_f = 1.3 \times 0.00016667 \approx 0.000217$
* **Control Output Formula**:
  $$\text{Power} = (K_f \cdot \text{ReferenceSpeed}) + (K_p \cdot \text{error}) + \text{IntegratedError}$$
  where $\text{error} = \text{ReferenceSpeed} - \text{CurrentSpeed}$ in RPM.
* **Non-Linear Integration**:
  To prevent overshoot during rapid accelerations, the integration rate is reduced when error is high:
  $$\text{if } |\text{error}| < 250\text{ RPM}: \quad \text{IntegratedError} += K_i \cdot \text{error} \cdot dt$$
  $$\text{if } |\text{error}| \ge 250\text{ RPM}: \quad \text{IntegratedError} += K_i \cdot 0.15 \cdot \text{error} \cdot dt$$
* **Anti-Windup Clamping**:
  If $\text{error} > 400\text{ RPM}$ and $\text{IntegratedError} < 0$, or $\text{error} < -400\text{ RPM}$ and $\text{IntegratedError} > 0$, `IntegratedError` is immediately clamped to zero.

---

## 3. Odometry Subsystem (`Subsystems/Odometry/`)

The odometry architecture separates low-level hardware tracking from high-level sensor fusion across three classes:

### 3.1 `PinpointOdometry.java` (Dead-Wheel Tracking Computer)
Communicates with the **goBILDA Pinpoint Odometry Computer** hardware device over I2C.
* **Encoders**: Uses goBILDA 4-bar swingarm pods (`goBILDA_SWINGARM_POD`).
* **Offsets**: X offset $+44.0\text{ mm}$, Y offset $+108.0\text{ mm}$ relative to the robot's center of rotation.
* **Update Cycle**: Calls `pinpointDriver.update()` in `periodic()`.
* **Output**: `GetPose()` returns an FTCLib `Pose2d` containing $(X, Y)$ in meters and heading $\theta$ in radians.

### 3.2 `Gyro.java` (Rev Control Hub IMU)
Provides supplementary/backup heading and tilt data directly from the internal BNO055/IMU inside the Rev Control Hub:
* **Orientation**: Configured with `LogoFacingDirection.BACKWARD` and `UsbFacingDirection.UP`.
* **Outputs**: Filtered `YawAngle` (heading) and `RollAngle` (chassis tilt) in degrees, supporting runtime offsets and zeroing.

### 3.3 `Odometry.java` (Main Coordinator & Vision Fusion)
`Odometry` acts as the single source of truth for the robot's global position. In every loop iteration:

1. **Dead-Wheel Read**: Fetches the high-frequency pose estimate from `RobotContainer.odometryPod.GetPose()`.
2. **AprilTag Vision Gating**: Queries `RobotContainer.limeLight` for MegaTag2 localization results. Detections are validated against alliance targets:
   * **Red Alliance**: AprilTag ID `24`
   * **Blue Alliance**: AprilTag ID `20`
3. **Speed-Gated Complementary Filter**:
   To prevent motion blur and vision latency artifacts, vision updates are only accepted when:
   * The robot is in **TeleOp** mode (`RobotContainer.Modes.TeleOp`).
   * The reported vision position is within the physical field boundaries ($-1.8\text{ m} < x, y < 1.8\text{ m}$).
   * All four wheel speeds from `DriveTrain.GetWheelSpeeds()` are between $-0.1\text{ m/s}$ and $+0.1\text{ m/s}$ (robot is stationary or creeping).
   
   When conditions are met, a complementary low-pass filter slowly corrects odometry drift:
   $$\text{Pose}_X = 0.99 \cdot \text{DeadWheel}_X + 0.01 \cdot \text{Limelight}_X$$
   $$\text{Pose}_Y = 0.99 \cdot \text{DeadWheel}_Y + 0.01 \cdot \text{Limelight}_Y$$
   $$\text{Pose}_\theta = \text{GyroHeading}$$
   
   The filtered pose is written back to the Pinpoint computer via `pinpointDriver.setPosition()`, preventing future dead-wheel integration from accumulating that drift.

4. **Cross-OpMode Pose Persistence**:
   The robot's pose is saved to a static variable `StoredRobotPose` on every loop. When transitioning from Autonomous to TeleOp, the newly initialized `Odometry` subsystem loads `StoredRobotPose`, preserving the field coordinate frame without needing to re-zero at the start of driver control.

---

## 4. Subsystem Interoperability Summary

| Interaction | Direction | Mechanism |
| :--- | :--- | :--- |
| **Field-Oriented Driving** | `Gyro` $\rightarrow$ `DriveTrain` | `DriveTrain.FieldDrive()` reads `RobotContainer.gyro.getYawAngle()` to transform field vectors into robot chassis vectors. |
| **Vision Gating** | `DriveTrain` $\rightarrow$ `Odometry` | `Odometry.periodic()` inspects `DriveTrain.GetWheelSpeeds()` to ensure linear wheel velocity $< 0.1\text{ m/s}$ before applying vision corrections. |
| **Autonomous Pathing** | `Odometry` $\rightarrow$ `DriveTrain` | Autonomous commands read `Odometry.getCurrentPos()` to calculate trajectory errors and supply velocity commands to `DriveTrain.FieldDrive()` / `DriveTrain.RobotDrive()`. |
| **Heading Alignment** | `Odometry` $\rightarrow$ `Gyro` | Calling `Odometry.setCurrentPos(Pose2d)` updates both the Pinpoint tracking computer and aligns `Gyro.setYawAngle()`. |
