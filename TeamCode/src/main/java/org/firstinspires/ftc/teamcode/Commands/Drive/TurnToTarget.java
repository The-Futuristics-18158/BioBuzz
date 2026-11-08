package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * Command that calculates the field angle toward the alliance goal target from current odometry,
 * and uses adaptive gain PID control with low-pass filtered error to turn the robot to face the target.
 *
 * @author gbuckholtz
 */
public class TurnToTarget extends CommandBase {

    private double m_endangle;
    private double m_timeout;

    // Rotation speed and error
    private double m_rotatespeed;
    private double m_angleerror;

    // Adaptive PID gains for rotation toward target
    public static double kpmax = 0.11;
    public static double kpmin = 0.04;
    public static double kp_deg = 90.0;
    public static double ki = 0.2;
    public static double ki_range = 5.0;
    public static double ki_zerorange = 0.5;
    public static double kd = 0.019;
    public static double kd_deg = 110.0;

    // Low-pass filtered error angle
    private double filtered_error;
    public static double LPFvalue = 0.93;

    private PIDController pidController = new PIDController(kpmax, ki, kd);

    // Timers
    private ElapsedTime OnTargetTime;
    private ElapsedTime TimeOutTime;

    /**
     * Constructs a TurnToTarget command with a specified maximum timeout.
     *
     * @author gbuckholtz
     *
     * @param timeout maximum execution duration in seconds before timing out
     */
    public TurnToTarget(double timeout) {
        // Require drivetrain subsystem
        addRequirements(RobotContainer.drivesystem);

        m_timeout = timeout;
        OnTargetTime = new ElapsedTime();
        TimeOutTime = new ElapsedTime();
    }

    /**
     * Computes the initial target angle vector from odometry to the alliance goal, resets PID and timers.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Current robot position and goal target
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = new Translation2d();

        // Calculate target heading angle in degrees
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // Reset controller and timers
        pidController.reset();
        m_angleerror = 0.0;
        OnTargetTime.reset();
        TimeOutTime.reset();

        // Initial angle error
        filtered_error = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());
    }

    /**
     * Periodically calculates angle error, applies adaptive proportional and derivative gains,
     * and drives robot rotation.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Compute angle error
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // Apply low-pass filter to error
        filtered_error = LPFvalue * filtered_error + (1.0 - LPFvalue) * m_angleerror;

        // Compute adaptive proportional gain based on distance to target
        double pgain = kpmax - (kpmax - kpmin) * Math.abs(m_angleerror) / kp_deg;
        pgain = Math.max(kpmin, pgain);
        pgain = Math.min(kpmax, pgain);
        pidController.setP(pgain);

        // Enable integral action only within tight error band
        if (Math.abs(m_angleerror) < ki_range)
            pidController.setI(ki);
        else
            pidController.setI(0.0);

        // Reset integrator when very close to target to prevent oscillation
        if (Math.abs(m_angleerror) < ki_zerorange)
            pidController.reset();

        // Compute derivative gain scaled by filtered error
        double gain = Math.max(0.0, kd * (1.0 - Math.abs(filtered_error) / kd_deg));
        pidController.setD(gain);

        // Compute PID rotational velocity
        m_rotatespeed = pidController.calculate(m_angleerror);

        // Command robot rotation
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, m_rotatespeed);

        // Reset on-target settling timer if error exceeds 1 degree
        if (Math.abs(m_angleerror) >= 1.0)
            OnTargetTime.reset();
    }

    /**
     * Stops the robot drive when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);
    }

    /**
     * Determines whether the robot has maintained target alignment for >= 0.25 seconds or timed out.
     *
     * @author gbuckholtz
     *
     * @return true when target alignment duration or timeout is met
     */
    @Override
    public boolean isFinished() {
        return (OnTargetTime.seconds() > 0.25 || TimeOutTime.seconds() >= m_timeout);
    }
}