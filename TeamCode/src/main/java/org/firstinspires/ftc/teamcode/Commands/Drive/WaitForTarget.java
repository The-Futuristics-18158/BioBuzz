package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * Command that waits passively until the robot's heading aligns with the target goal vector
 * (settled within 1 degree for at least 0.25 seconds) or until a maximum timeout elapses.
 *
 * @author gbuckholtz
 */
public class WaitForTarget extends CommandBase {

    private double m_endangle;
    private double m_timeout;
    private double m_angleerror;

    // Timers
    private ElapsedTime OnTargetTime;
    private ElapsedTime TimeOutTime;

    /**
     * Constructs a WaitForTarget command with a specified maximum timeout.
     *
     * @author gbuckholtz
     *
     * @param timeout maximum wait duration in seconds before completing
     */
    public WaitForTarget(double timeout) {
        m_timeout = timeout;
        OnTargetTime = new ElapsedTime();
        TimeOutTime = new ElapsedTime();
    }

    /**
     * Computes the target heading angle to the goal and resets alignment and timeout timers.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Read current odometry pose
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = new Translation2d();

        // Calculate target heading angle in degrees
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // Reset timers
        OnTargetTime.reset();
        TimeOutTime.reset();
    }

    /**
     * Periodically monitors heading error and resets the on-target timer if error exceeds 1 degree.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Calculate angle error
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // Reset settling timer if error is 1.0 degree or greater
        if (Math.abs(m_angleerror) >= 1.0)
            OnTargetTime.reset();
    }

    /**
     * Called when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Determines whether the robot has settled on the target heading for >= 0.25s or timed out.
     *
     * @author gbuckholtz
     *
     * @return true when alignment duration or timeout is met
     */
    @Override
    public boolean isFinished() {
        return (OnTargetTime.seconds() > 0.25 || TimeOutTime.seconds() >= m_timeout);
    }
}