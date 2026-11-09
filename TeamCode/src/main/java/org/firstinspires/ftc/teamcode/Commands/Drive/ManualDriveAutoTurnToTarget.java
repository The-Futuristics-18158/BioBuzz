package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * TeleOp drive command combining manual translational drive with automated target tracking.
 * Translates according to driver joystick commands while continuously calculating and rotating
 * the robot heading to aim directly at the active alliance goal.
 *
 * @author gbuckholtz
 */
public class ManualDriveAutoTurnToTarget extends CommandBase {

    private double powerFactor;
    private double basePowerFacter = 0.65;
    private double boostPowerFacter = 0.35;
    private double slowPowerFactor = 0.4;
    private double m_endangle;

    // Heading error
    private double m_angleerror;

    // Adaptive PID gains for automated target rotation
    public static double kpmax = 0.13;
    public static double kpmin = 0.05;
    public static double kp_deg = 90.0;
    public static double ki = 0.2;
    public static double ki_range = 5.0;
    public static double ki_zerorange = 0.5;
    public static double kd = 0.019;
    public static double kd_deg = 110.0;

    // Low-pass filtered angle error
    private double filtered_error;
    public static double LPFvalue = 0.93;

    private PIDController pidController = new PIDController(kpmax, ki, kd);

    /**
     * Constructs a ManualDriveAutoTurnToTarget command requiring the drivetrain.
     *
     * @author gbuckholtz
     */
    public ManualDriveAutoTurnToTarget() {
        // Require drivetrain
        addRequirements(RobotContainer.drivesystem);
    }

    /**
     * Calculates initial target heading to goal and resets PID controllers.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Calculate goal target vector from odometry
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = new Translation2d();

        // Calculate heading in degrees
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // Reset controller
        pidController.reset();
        m_angleerror = 0.0;

        // Initialize filtered error
        filtered_error = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());
    }

    /**
     * Periodically processes translational joystick inputs and applies automated rotational PID control to track target.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Read joystick inputs
        double dX = RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        double dY = -RobotContainer.ActiveOpMode.gamepad1.left_stick_y;
        double speedTrigger = RobotContainer.ActiveOpMode.gamepad1.right_trigger;
        double slowTrigger = RobotContainer.ActiveOpMode.gamepad1.left_trigger;

        // Invert X and Y for blue alliance
        if (!RobotContainer.isRedAlliance) {
            dX = dX * -1;
            dY = dY * -1;
        }

        // Apply dead-zoning
        dX = Math.abs(dX) > 0.05 ? dX : 0;
        dY = Math.abs(dY) > 0.05 ? dY : 0;

        // Compute power factor
        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter) - (slowTrigger * slowPowerFactor);
        powerFactor = powerFactor * RobotContainer.drivesystem.MAX_SPEED;

        // Scale translation velocities
        dX *= powerFactor;
        dY *= powerFactor;

        // Automated aim calculation
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = new Translation2d();
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // Compute error and low-pass filter
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());
        filtered_error = LPFvalue * filtered_error + (1.0 - LPFvalue) * m_angleerror;

        // Adaptive proportional gain
        double pgain = kpmax - (kpmax - kpmin) * Math.abs(m_angleerror) / kp_deg;
        pgain = Math.max(kpmin, pgain);
        pgain = Math.min(kpmax, pgain);
        pidController.setP(pgain);

        // Clamped integral action
        if (Math.abs(m_angleerror) < ki_range)
            pidController.setI(ki);
        else
            pidController.setI(0.0);

        // Zero integrator near zero error
        if (Math.abs(m_angleerror) < ki_zerorange)
            pidController.reset();

        // Scaled derivative gain
        double gain = Math.max(0.0, kd * (1.0 - Math.abs(filtered_error) / kd_deg));
        pidController.setD(gain);

        // Compute automated rotation
        double omega = pidController.calculate(m_angleerror);

        // Command field-oriented drive
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega);
    }

    /**
     * Runs continuously during TeleOp until canceled.
     *
     * @author gbuckholtz
     *
     * @return always false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Halts drivetrain when command ends or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by scheduler
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);
    }
}