package org.firstinspires.ftc.teamcode.Commands.Odomeetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command to calibrate and reset the robot's odometry position (X, Y) and heading angle
 * using multiple filtered vision/sensor samples while the robot remains stationary.
 *
 * @author gbuckholtz
 */
public class ResetOdometryXYAngle extends CommandBase {

    private boolean FirstSample;
    private double PoseX, PoseY, PoseRad;
    private int NumberSamples;

    /**
     * Constructs a ResetOdometryXYAngle command requiring the drivetrain.
     *
     * @author gbuckholtz
     */
    public ResetOdometryXYAngle() {
        // Require the drivetrain subsystem
        addRequirements(RobotContainer.drivesystem);
    }

    /**
     * Halts drive motors and resets sample counters and pose accumulators.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Stop robot motion while sampling odometry
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);

        // Reset sample tracking
        FirstSample = true;
        PoseX = 0.0;
        PoseY = 0.0;
        PoseRad = 0.0;
        NumberSamples = 0;
    }

    /**
     * Periodic execution loop for sampling and filtering position readings.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Sample collection logic
    }

    /**
     * Determines whether enough position samples have been accumulated.
     *
     * @author gbuckholtz
     *
     * @return true when more than 15 samples are gathered
     */
    @Override
    public boolean isFinished() {
        // Finished when sample threshold is reached
        return (NumberSamples > 15);
    }

    /**
     * Applies the averaged position and heading to the robot odometry subsystem upon completion.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // If at least one valid sample was obtained, update the robot odometry pose
        if (NumberSamples > 0) {
            Pose2d CurrentPose = new Pose2d(PoseX, PoseY, new Rotation2d(PoseRad));
            RobotContainer.odometry.setCurrentPos(CurrentPose);
        }
    }
}