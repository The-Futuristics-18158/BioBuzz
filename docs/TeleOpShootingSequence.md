# Blue TeleOp shooting: three scenarios

These diagrams explain what `AutomaticShootCommand` would do if Blue TeleOp's
right bumper were connected to it. **That connection is hypothetical**: the
current right bumper runs `ShotSequence` instead. No robot code is changed by
these diagrams.

Read each diagram from top to bottom. An arrow shows one part of the program
asking another part to do something or returning an answer.

- **CommandScheduler** starts commands and runs them repeatedly.
- **AutomaticShootCommand** decides when to check, wait, kick, and stop.
- **Subsystems** control or read parts of the robot: the ball sensor, flywheel
  gap servo, flywheel motors, and kicker paddles.
- **RPM** means revolutions per minute, a measure of how fast a wheel spins.

The examples use the command's default settings and assume the driver keeps
holding the button until the command finishes.

## Command lifecycle: Nectar inside the control loop

The blue box is the **control loop**: the robot repeats it approximately every
20 milliseconds throughout TeleOp, even after the command finishes. The yellow
box shows the Nectar shooting work inside that loop. Each pass through the loop
runs only the command's current state, not all three states at once.

```mermaid
sequenceDiagram
    actor Driver
    participant Blue as BlueTeleop
    participant Robot as RobotContainer
    participant Scheduler as CommandScheduler
    participant Shoot as AutomaticShootCommand
    participant Sensor as BallColorSensor
    participant Gap as FlywheelGap
    participant Wheel as Flywheel
    participant Kicker

    Driver->>Blue: Hold right bumper (hypothetical binding)
    rect rgb(225, 240, 255)
        loop Control loop: approximately every 20 ms during TeleOp
            Blue->>Robot: run() calls Periodic()
            Robot->>Scheduler: run()
            Scheduler->>Sensor: periodic() updates ball reading
            Scheduler->>Gap: periodic() applies gap target
            Scheduler->>Wheel: periodic() updates wheel reading
            Scheduler->>Kicker: periodic() updates fin readings
            rect rgb(255, 249, 210)
                opt Bumper just pressed
                    Scheduler->>Shoot: initialize() once
                    Shoot->>Sensor: Check presence and ball type
                    Sensor-->>Shoot: Ball present and NECTAR
                    Shoot->>Gap: Request Nectar gap (0.02)
                    Shoot->>Wheel: Request 4500 RPM
                    Shoot->>Shoot: State becomes WAIT_FOR_READY
                end
                alt Bumper released while command is active
                    Scheduler->>Shoot: end(true) - interrupted
                    Shoot->>Kicker: stop()
                    Shoot->>Wheel: SetFlywheelSpeed(0)
                    Shoot->>Gap: Reset gap to 0.02
                else Command is still scheduled
                    Scheduler->>Shoot: execute() once this loop
                    alt WAIT_FOR_READY
                        Shoot->>Wheel: Check speed
                        Note over Shoot: When ready or after 2 seconds, and after at least 0.15 seconds
                        Shoot->>Kicker: advanceToShoot() when ready to proceed
                        Shoot->>Shoot: Change to ADVANCE_KICKER
                    else ADVANCE_KICKER
                        Note over Shoot: Once 0.40 seconds have passed
                        Shoot->>Kicker: clearFeedPath()
                        Shoot->>Shoot: Change to CLEAR_FINS
                    else CLEAR_FINS
                        Shoot->>Kicker: Check whether feed path is clear
                        Note over Shoot: When clear or after 1 second
                        Shoot->>Kicker: stop()
                        Shoot->>Shoot: Change to COMPLETED
                    end
                    Scheduler->>Shoot: isFinished()
                    Shoot-->>Scheduler: Yes only when COMPLETED
                    opt Command is finished
                        Scheduler->>Shoot: end(false) - completed normally
                        Shoot->>Kicker: stop()
                        Shoot->>Wheel: SetFlywheelSpeed(0)
                        Shoot->>Gap: Reset gap to 0.02
                    end
                end
            end
        end
    end
```

`initialize()` runs once when scheduled. `execute()` and `isFinished()` run on
each scheduler pass while the command remains active. `end()` runs once when
the command finishes or is interrupted. The detailed Nectar diagram below
shows the same shot without the surrounding lifecycle.

## 1. Nectar ball

```mermaid
sequenceDiagram
    autonumber
    actor Driver
    participant Blue as BlueTeleop
    participant Scheduler as CommandScheduler
    participant Shoot as AutomaticShootCommand
    participant Sensor as BallColorSensor
    participant Gap as FlywheelGap
    participant Wheel as Flywheel
    participant Kicker

    Driver->>Blue: Hold right bumper (hypothetical binding)
    Blue->>Scheduler: RobotContainer runs the scheduler
    Scheduler->>Sensor: Update cached sensor readings
    Scheduler->>Shoot: Start command
    Shoot->>Sensor: isBallPresent()
    Sensor-->>Shoot: Yes
    Shoot->>Sensor: getBallType()
    Sensor-->>Shoot: NECTAR
    Shoot->>Gap: setGapForBall(NECTAR)
    Note over Gap: Request wider gap (servo position 0.02).
    Shoot->>Wheel: SetFlywheelSpeed(4500 RPM)
    loop Until wheel is ready or 2 seconds have passed
        Scheduler->>Shoot: Run command again
        Shoot->>Wheel: isAtTargetSpeed(100 RPM)
        Wheel-->>Shoot: Ready or not ready
    end
    Note over Shoot: Also wait at least 0.15 seconds for the gap servo. The two waits overlap.
    Shoot->>Kicker: advanceToShoot()
    Note over Kicker: Paddles attempt to push the ball into the flywheels.
    Note over Shoot: Wait 0.40 seconds for the kick.
    Shoot->>Kicker: clearFeedPath()
    loop Until both fins are clear or 1 second has passed
        Scheduler->>Kicker: Update side sensor readings
        Shoot->>Kicker: isFeedPathClear()
        Kicker-->>Shoot: Clear or not clear
    end
    Shoot->>Kicker: stop()
    Scheduler->>Shoot: Finish command
    Shoot->>Kicker: stop()
    Shoot->>Wheel: SetFlywheelSpeed(0)
    Shoot->>Gap: Reset gap to 0.02
```

## 2. Pollen ball

```mermaid
sequenceDiagram
    autonumber
    actor Driver
    participant Blue as BlueTeleop
    participant Scheduler as CommandScheduler
    participant Shoot as AutomaticShootCommand
    participant Sensor as BallColorSensor
    participant Gap as FlywheelGap
    participant Wheel as Flywheel
    participant Kicker

    Driver->>Blue: Hold right bumper (hypothetical binding)
    Blue->>Scheduler: RobotContainer runs the scheduler
    Scheduler->>Sensor: Update cached sensor readings
    Scheduler->>Shoot: Start command
    Shoot->>Sensor: isBallPresent()
    Sensor-->>Shoot: Yes
    Shoot->>Sensor: getBallType()
    Sensor-->>Shoot: POLLEN
    Shoot->>Gap: setGapForBall(POLLEN)
    Note over Gap: Request narrower gap (servo position 0.08).
    Shoot->>Wheel: SetFlywheelSpeed(4500 RPM)
    loop Until wheel is ready or 2 seconds have passed
        Scheduler->>Shoot: Run command again
        Shoot->>Wheel: isAtTargetSpeed(100 RPM)
        Wheel-->>Shoot: Ready or not ready
    end
    Note over Shoot: Also wait at least 0.15 seconds for the gap servo. The two waits overlap.
    Shoot->>Kicker: advanceToShoot()
    Note over Kicker: Paddles attempt to push the ball into the flywheels.
    Note over Shoot: Wait 0.40 seconds for the kick.
    Shoot->>Kicker: clearFeedPath()
    loop Until both fins are clear or 1 second has passed
        Scheduler->>Kicker: Update side sensor readings
        Shoot->>Kicker: isFeedPathClear()
        Kicker-->>Shoot: Clear or not clear
    end
    Shoot->>Kicker: stop()
    Scheduler->>Shoot: Finish command
    Shoot->>Kicker: stop()
    Shoot->>Wheel: SetFlywheelSpeed(0)
    Shoot->>Gap: Reset gap to 0.02
```

## 3. No ball detected

```mermaid
sequenceDiagram
    autonumber
    actor Driver
    participant Blue as BlueTeleop
    participant Scheduler as CommandScheduler
    participant Shoot as AutomaticShootCommand
    participant Sensor as BallColorSensor
    participant Gap as FlywheelGap
    participant Wheel as Flywheel
    participant Kicker

    Driver->>Blue: Hold right bumper (hypothetical binding)
    Blue->>Scheduler: RobotContainer runs the scheduler
    Scheduler->>Sensor: Update cached sensor readings
    Scheduler->>Shoot: Start command
    Shoot->>Sensor: isBallPresent()
    Sensor-->>Shoot: No
    Shoot->>Shoot: Mark command completed
    Note over Shoot,Kicker: No kick is requested.
    Scheduler->>Shoot: Finish command
    Shoot->>Kicker: stop()
    Shoot->>Wheel: SetFlywheelSpeed(0)
    Shoot->>Gap: Reset gap to 0.02
```

## Things to know

- The sensor stores its latest reading for the command to use. If it detects
  a ball but cannot identify its type, the command uses the wider Nectar gap.
- The command can kick after the 2-second timeout even if the wheel is not
  ready. A clear feed path means both cached side-sensor distances are at least
  60 mm; a timeout also ends the wait.
- The current flywheel code calculates motor power but does **not** apply it
  during TeleOp. These diagrams show the requested actions, not a guarantee
  that a ball is launched. Other periodic robot code can also change the
  requested speed and gap.
- With the hypothetical `whenHeld` binding, releasing the bumper early
  interrupts the command and runs its stop/reset actions. Keeping it held after
  completion does not start another shot.

## Source

- [BlueTeleop](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/TeleOp/BlueTeleop.java),
  [RobotContainer](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotContainer.java)
- [AutomaticShootCommand](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Commands/Shoot/AutomaticShootCommand.java)
- [BallColorSensor](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Sensors/BallColorSensor.java),
  [FlywheelGap](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Shooter/FlywheelGap.java),
  [Flywheel](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Shooter/Flywheel.java),
  [Kicker](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Shooter/Kicker.java)
