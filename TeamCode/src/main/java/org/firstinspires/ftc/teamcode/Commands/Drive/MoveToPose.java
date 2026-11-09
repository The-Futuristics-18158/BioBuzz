package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.ArrayList;

/**
 * Command that moves the robot in a straight line from its current odometry pose
 * to a specified target Pose2d, using a generated FollowPath trajectory with defined speed and acceleration limits.
 *
 * @author gbuckholtz
 */
public class MoveToPose extends CommandBase {

    // Internal FollowPath instance delegated to execute the trajectory
    private FollowPath cmd;

    // Trajectory parameters
    private double maxSpeed;
    private double maxAccel;
    private Pose2d dest;

    /**
     * Constructs a MoveToPose command.
     *
     * @author gbuckholtz
     *
     * @param maxSpeed maximum linear travel speed in meters per second
     * @param maxAccel maximum linear acceleration in meters per second squared
     * @param destination target destination Pose2d (X, Y in meters, rotation)
     */
    public MoveToPose(double maxSpeed,
                      double maxAccel,
                      Pose2d destination) {
        this.maxSpeed = maxSpeed;
        this.maxAccel = maxAccel;
        this.dest = destination;

        // Requires drivetrain
        addRequirements(RobotContainer.drivesystem);
    }

    /**
     * Calculates the straight-line heading from current position to destination,
     * constructs the FollowPath command, and initializes it.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Read current robot pose from odometry
        Pose2d currentPose = RobotContainer.odometry.getCurrentPos();

        // Calculate straight-line heading vector
        Rotation2d heading = new Rotation2d(dest.getX() - currentPose.getX(),
                dest.getY() - currentPose.getY());

        // Construct underlying FollowPath command
        cmd = new FollowPath(maxSpeed,
                maxAccel,
                0.0,
                0.0,
                heading,
                new ArrayList<Translation2d>(),
                new Pose2d(dest.getX(), dest.getY(), heading),
                dest.getRotation());

        // Initialize path following
        cmd.initialize();
    }

    /**
     * Periodically executes the delegated FollowPath command.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        cmd.execute();
    }

    /**
     * Determines whether the straight-line trajectory has completed.
     *
     * @author gbuckholtz
     *
     * @return true when the underlying FollowPath command is finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    /**
     * Finishes or interrupts the underlying FollowPath command.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }
}