package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * Command to rotate the robot to a specified target heading (either relative to current heading
 * or absolute field-oriented) using closed-loop PID rotational control and an on-target settling timer.
 *
 * @author gbuckholtz
 */
public class TurnTo extends CommandBase {

    private double m_angle;
    private boolean m_relative;
    private double m_endangle;
    private double m_timeout;

    // Rotation speed and error calculated by PID
    private double m_rotatespeed;
    private double m_angleerror;

    // PID controller gains for heading rotation
    private double kp = 0.075;
    private double ki = 0.2;
    private double kd = 0.0025;
    private PIDController pidController = new PIDController(kp, ki, kd);

    // On-target settling timer and overall timeout timer
    private ElapsedTime OnTargetTime;
    private ElapsedTime TimeOutTime;

    /**
     * Constructs a TurnTo command.
     *
     * @author gbuckholtz
     *
     * @param angle target angle in degrees (-180 to 180)
     * @param relative true if angle is relative to current robot heading; false if absolute field heading
     * @param timeout maximum duration in seconds before forcing command completion
     */
    public TurnTo(double angle, boolean relative, double timeout) {
        // Require drivetrain
        addRequirements(RobotContainer.drivesystem);

        m_angle = angle;
        m_relative = relative;
        m_timeout = timeout;

        OnTargetTime = new ElapsedTime();
        TimeOutTime = new ElapsedTime();
    }

    /**
     * Computes the target ending angle, resets the PID controller, and starts timers.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Current heading from gyro
        double ourcurrentangle = RobotContainer.gyro.getYawAngle();

        // Calculate target end angle
        if (m_relative)
            m_endangle = ourcurrentangle + m_angle;
        else
            m_endangle = m_angle;

        // Reset controller and timers
        pidController.reset();
        m_angleerror = 0.0;
        OnTargetTime.reset();
        TimeOutTime.reset();
    }

    /**
     * Periodically computes heading error via Utils.AngleDifference, updates PID output, and commands rotation.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Compute wrapped angle difference
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // Only integrate when within 10 degrees of target to prevent windup
        if (Math.abs(m_angleerror) < 10.0)
            pidController.setI(ki);
        else
            pidController.setI(0.0);

        // Compute rotational velocity from PID
        m_rotatespeed = pidController.calculate(m_angleerror);

        // Apply rotation to drivetrain
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, m_rotatespeed);

        // Reset on-target timer if error exceeds 3 degrees
        if (Math.abs(m_angleerror) >= 3.0)
            OnTargetTime.reset();
    }

    /**
     * Halts drivetrain rotation when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // Stop robot drive
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);
    }

    /**
     * Determines whether the robot has settled on target for >= 0.25 seconds, or if the timeout has elapsed.
     *
     * @author gbuckholtz
     *
     * @return true when on-target condition or timeout is satisfied
     */
    @Override
    public boolean isFinished() {
        return (OnTargetTime.seconds() > 0.25 || TimeOutTime.seconds() >= m_timeout);
    }
}