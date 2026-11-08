package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Mecanum drive train subsystem.
 * Handles robot kinematics, field-oriented and robot-oriented driving,
 * and individual closed-loop motor PIF speed control.
 *
 * @author gbuckholtz
 */
public class DriveTrain extends SubsystemBase {

    // Constants for DC Motor
    final double MAXRPM = 6000.0;
    final double MAXRPS = MAXRPM / 60.0;

    // Motor speed ticks per revolution to m/s travel speed
    final double TICKS_PER_ROTATION = (28.0);       // Encoder pulses per motor revolution
    final double MAX_SPEED_TICKS_PER_SEC = MAXRPS * TICKS_PER_ROTATION;
    final double WHEEL_DIA = 0.104;                 // Wheel diameter in meters
    final double GEAR_RATIO = 20.0;                 // Drive gear ratio
    final double TICKSPS_TO_MPS = WHEEL_DIA * Math.PI / TICKS_PER_ROTATION / GEAR_RATIO;
    final double MPS_TO_TICKSPS = 1.0 / TICKSPS_TO_MPS;
    public final double MAX_SPEED = MAXRPS * Math.PI * WHEEL_DIA / GEAR_RATIO;

    // Kinematics physical dimensions in meters
    final double TRACK_WIDTH = 0.34;   // Width between left and right wheels in meters
    final double TRACK_LENGTH = 0.192; // Length between front and back wheels in meters
    private MecanumDriveKinematics driveKinematics;

    // Mecanum drive motor hardware references
    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    // Individual motor PIF controllers
    private MotorControl leftFrontControl;
    private MotorControl leftBackControl;
    private MotorControl rightFrontControl;
    private MotorControl rightBackControl;

    // Reference wheel speeds for closed-loop control in m/s
    MecanumDriveWheelSpeeds ReferenceWheelSpeeds;

    /**
     * Initializes the drive train subsystem, sets up mecanum kinematics,
     * configures motor directions and brake behaviors, and instantiates motor controllers.
     *
     * @author gbuckholtz
     */
    public DriveTrain() {
        // Setup Mecanum kinematics with wheel offset locations relative to robot center
        driveKinematics = new MecanumDriveKinematics(
                // Front Left
                new Translation2d(TRACK_LENGTH * 0.208, TRACK_WIDTH * 0.5),
                // Front Right
                new Translation2d(TRACK_LENGTH * 0.208, TRACK_WIDTH * -0.5),
                // Back Left
                new Translation2d(TRACK_LENGTH * -0.792, TRACK_WIDTH * 0.5),
                // Back Right
                new Translation2d(TRACK_LENGTH * -0.792, TRACK_WIDTH * -0.5));

        // Retrieve drive motors from the active OpMode's hardware map
        leftFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        // Set motor directions (right side reversed for forward driving)
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run without internal encoder control (closed-loop handled in software periodic)
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize motor powers to zero
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        // Configure active braking on zero power
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Create PIF controllers for each drive motor
        leftFrontControl = new MotorControl();
        leftBackControl = new MotorControl();
        rightFrontControl = new MotorControl();
        rightBackControl = new MotorControl();

        // Default reference wheel speeds to zero
        ReferenceWheelSpeeds = new MecanumDriveWheelSpeeds(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Method called periodically by the command scheduler to compute and apply closed-loop PIF speed control to each drive motor.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Convert motor encoder velocity ticks/s to RPM
        double TickstoRPM = 60.0 / TICKS_PER_ROTATION;
        double CurrentLeftFrontRPM = TickstoRPM * leftFrontDrive.getVelocity();
        double CurrentRightFrontRPM = TickstoRPM * rightFrontDrive.getVelocity();
        double CurrentLeftBackRPM = TickstoRPM * leftBackDrive.getVelocity();
        double CurrentRightBackRPM = TickstoRPM * rightBackDrive.getVelocity();

        // Convert reference linear wheel speeds (m/s) to motor RPM
        double MPStoRPM = GEAR_RATIO * 60.0 / (WHEEL_DIA * Math.PI);
        double RefLeftFrontRPM = MPStoRPM * ReferenceWheelSpeeds.frontLeftMetersPerSecond;
        double RefRightFrontRPM = MPStoRPM * ReferenceWheelSpeeds.frontRightMetersPerSecond;
        double RefLeftBackRPM = MPStoRPM * ReferenceWheelSpeeds.rearLeftMetersPerSecond;
        double RefRightBackRPM = MPStoRPM * ReferenceWheelSpeeds.rearRightMetersPerSecond;

        // Compute PIF outputs and apply powers to each drive motor
        leftFrontDrive.setPower(leftFrontControl.calculate(RefLeftFrontRPM, CurrentLeftFrontRPM));
        rightFrontDrive.setPower(rightFrontControl.calculate(RefRightFrontRPM, CurrentRightFrontRPM));
        leftBackDrive.setPower(leftBackControl.calculate(RefLeftBackRPM, CurrentLeftBackRPM));
        rightBackDrive.setPower(rightBackControl.calculate(RefRightBackRPM, CurrentRightBackRPM));
    }

    /**
     * Drives the robot in field-oriented coordinates.
     *
     * @author gbuckholtz
     *
     * @param Vx forward/backward velocity in meters per second
     * @param Vy left/right velocity in meters per second
     * @param Omega rotational velocity in radians per second
     */
    public void FieldDrive(double Vx, double Vy, double Omega) {
        // Read gyro heading angle in radians
        double rotAngRad = Math.toRadians(RobotContainer.gyro.getYawAngle());

        // Rotate field-centric vector by negative gyro angle to transform to robot coordinates
        double x = Vx * Math.cos(-rotAngRad) - Vy * Math.sin(-rotAngRad);
        double y = Vx * Math.sin(-rotAngRad) + Vy * Math.cos(-rotAngRad);

        // Drive with robot-centric velocities
        RobotDrive(x, y, Omega);
    }

    /**
     * Drives the robot in robot-oriented coordinates.
     *
     * @author gbuckholtz
     *
     * @param Vx forward/backward velocity in meters per second
     * @param Vy left/right velocity in meters per second
     * @param Omega rotational velocity in radians per second
     */
    public void RobotDrive(double Vx, double Vy, double Omega) {
        // Create ChassisSpeeds object from velocity commands
        ChassisSpeeds driveChassisSpeeds = new ChassisSpeeds(Vx, Vy, Omega);

        // Calculate individual wheel speeds using inverse kinematics around robot center
        ReferenceWheelSpeeds = driveKinematics.toWheelSpeeds(driveChassisSpeeds, new Translation2d(0, 0));

        // Normalize wheel speeds if any wheel exceeds MAX_SPEED
        double max = Math.max(Math.abs(ReferenceWheelSpeeds.frontLeftMetersPerSecond), Math.abs(ReferenceWheelSpeeds.frontRightMetersPerSecond));
        max = Math.max(max, Math.abs(ReferenceWheelSpeeds.rearRightMetersPerSecond));
        max = Math.max(max, Math.abs(ReferenceWheelSpeeds.rearLeftMetersPerSecond));
        if (max > MAX_SPEED) {
            double factor = MAX_SPEED / max;
            ReferenceWheelSpeeds.frontLeftMetersPerSecond *= factor;
            ReferenceWheelSpeeds.frontRightMetersPerSecond *= factor;
            ReferenceWheelSpeeds.rearRightMetersPerSecond *= factor;
            ReferenceWheelSpeeds.rearLeftMetersPerSecond *= factor;
        }

        // Apply immediately to motors
        periodic();
    }

    /**
     * Retrieves the measured speeds of all four mecanum wheels in m/s.
     *
     * @author gbuckholtz
     *
     * @return MecanumDriveWheelSpeeds containing each wheel's speed in meters per second
     */
    public MecanumDriveWheelSpeeds GetWheelSpeeds() {
        // Convert motor encoder ticks/s to wheel travel m/s
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds();
        speeds.rearLeftMetersPerSecond = TICKSPS_TO_MPS * leftBackDrive.getVelocity();
        speeds.frontLeftMetersPerSecond = TICKSPS_TO_MPS * leftFrontDrive.getVelocity();
        speeds.rearRightMetersPerSecond = TICKSPS_TO_MPS * rightBackDrive.getVelocity();
        speeds.frontRightMetersPerSecond = TICKSPS_TO_MPS * rightFrontDrive.getVelocity();

        return speeds;
    }

    /**
     * Retrieves the defined mecanum drive kinematics object.
     *
     * @author gbuckholtz
     *
     * @return MecanumDriveKinematics representing the physical drivetrain geometry
     */
    public MecanumDriveKinematics GetKinematics() {
        return driveKinematics;
    }

    /**
     * Dedicated motor controller class implementing PIF (Proportional-Integral-Feedforward)
     * velocity control with anti-windup for a single drive motor.
     *
     * @author gbuckholtz
     */
    private class MotorControl {
        private double Pgain = 1.25 * 0.00016667;
        private double Igain = 5.5 * 0.00016667;
        private double Fgain = 1.3 * 0.00016667;

        // Timing and error accumulator
        private ElapsedTime intervalTime;
        private double IntegratedError;

        /**
         * Initializes the motor controller and resets timer and integrated error.
         *
         * @author gbuckholtz
         */
        private MotorControl() {
            intervalTime = new ElapsedTime();
            reset();
        }

        /**
         * Resets the controller timer and zeroes the accumulated integrated error.
         *
         * @author gbuckholtz
         */
        private void reset() {
            intervalTime.reset();
            IntegratedError = 0.0;
        }

        /**
         * Computes the motor power control output given target and actual RPM.
         *
         * @author gbuckholtz
         *
         * @param ReferenceSpeed target motor speed in RPM
         * @param CurrentSpeed actual measured motor speed in RPM
         * @return commanded motor power (-1.0 to 1.0)
         */
        private double calculate(double ReferenceSpeed, double CurrentSpeed) {
            // Measure time delta since last execution, clamped at 70ms
            double dt = intervalTime.seconds();
            if (dt > 0.07)
                dt = 0.07;
            intervalTime.reset();

            // Compute speed error in RPM
            double error = ReferenceSpeed - CurrentSpeed;

            // Non-linear integration: reduce integral action when error is large (> 250 RPM)
            if (Math.abs(error) < 250.0)
                IntegratedError += Igain * error * dt;
            else
                IntegratedError += Igain * 0.15 * error * dt;

            // Anti-windup clamping on large overshoots
            if (error > 400.0 && IntegratedError < 0.0)
                IntegratedError = 0.0;
            if (error < -400.0 && IntegratedError > 0.0)
                IntegratedError = 0.0;

            // Compute and return PIF control output
            return (Fgain * ReferenceSpeed) + (Pgain * error) + (IntegratedError);
        }
    }
}