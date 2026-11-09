package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.Trajectory.State;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

import java.util.List;

/**
 * Command to follow a smooth spline trajectory generated from waypoints and kinematics constraints.
 * Uses closed-loop feedback controllers for X, Y position and rotational heading to track path states in real time.
 *
 * @author gbuckholtz
 */
public class FollowPath extends CommandBase {

    // Trajectory Configuration
    private TrajectoryConfig config;

    // Generated Trajectory
    private Trajectory trajectory;

    // Trajectory path parameters
    private Rotation2d pathStartAngle;
    private Pose2d pathEndPose;
    private List<Translation2d> pathWaypoints;

    // Ending linear speed
    private double endSpeed;

    // Heading PID controller
    private PIDController xController, yController, thetaController;

    // Time elapsed on path
    private ElapsedTime pathTime;

    // Robot rotation tracking
    private Rotation2d robotEndAngle;
    private double startRobotAngle;
    private double rotationRate;

    // Integrated position errors
    private double x_ierror;
    private double y_ierror;

    /**
     * Constructs a FollowPath command configured with physical limits, waypoints, and endpoint specifications.
     *
     * @author gbuckholtz
     *
     * @param maxSpeed maximum linear robot travel speed in meters per second
     * @param maxAccel maximum linear robot travel acceleration in meters per second squared
     * @param startSpeed initial starting speed in meters per second (0.0 unless chaining)
     * @param endSpeed final speed at trajectory completion in meters per second (0.0 to stop)
     * @param pathStartAngle tangent departure angle of the path start
     * @param pathWaypoints intermediate interior translation waypoints (X, Y)
     * @param pathEndPose destination pose (position and tangent arrival heading)
     * @param robotEndAngle target final heading orientation of the robot body
     */
    public FollowPath(double maxSpeed,
                      double maxAccel,
                      double startSpeed,
                      double endSpeed,
                      Rotation2d pathStartAngle,
                      List<Translation2d> pathWaypoints,
                      Pose2d pathEndPose,
                      Rotation2d robotEndAngle) {
        // Require drivetrain subsystem
        addRequirements(RobotContainer.drivesystem);

        // Configure trajectory constraints
        config = new TrajectoryConfig(maxSpeed, maxAccel);
        config.setReversed(false);
        config.setStartVelocity(startSpeed);
        config.setEndVelocity(endSpeed);
        config.setKinematics(RobotContainer.drivesystem.GetKinematics());
        config.addConstraint(new MecanumDriveKinematicsConstraint(
                RobotContainer.drivesystem.GetKinematics(),
                RobotContainer.drivesystem.MAX_SPEED));

        // Save path endpoints and parameters
        this.pathStartAngle = pathStartAngle;
        this.pathEndPose = pathEndPose;
        this.pathWaypoints = pathWaypoints;
        this.robotEndAngle = robotEndAngle;
        this.endSpeed = endSpeed;

        // Configure controllers
        xController = new PIDController(6.0, 0.00, 0.0);
        yController = new PIDController(6.0, 0.00, 0.0);
        thetaController = new PIDController(6.0, 0.0, 0.0);

        // Clamp integration bounds
        xController.setIntegrationBounds(-2.5, 2.5);
        yController.setIntegrationBounds(-2.5, 2.5);
        thetaController.setIntegrationBounds(-5.0, 5.0);

        // Path timer
        pathTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Samples current position from odometry, generates the spline trajectory,
     * resets feedback controllers and path timer, and computes body rotation rate.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Sample current robot pose from odometry
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();

        // Construct path starting pose at current position with departure heading
        Pose2d pathStartPose = new Pose2d(currentPos.getTranslation(), pathStartAngle);

        // Generate trajectory spline
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(pathStartPose, pathWaypoints, pathEndPose, config);
        } catch (Exception e) {
            trajectory = null;
        }

        // Reset controllers
        xController.reset();
        yController.reset();
        thetaController.reset();

        // Zero error integrators
        x_ierror = 0.0;
        y_ierror = 0.0;

        // Reset path timer
        pathTime.reset();

        // Set initial body heading in radians
        startRobotAngle = currentPos.getRotation().getRadians();

        // Calculate body rotation rate in rad/s over the duration of the trajectory
        if (trajectory != null && trajectory.getTotalTimeSeconds() != 0.0) {
            rotationRate = Math.toRadians(Utils.AngleDifference(Math.toDegrees(startRobotAngle),
                    robotEndAngle.getDegrees())) / trajectory.getTotalTimeSeconds();
        } else {
            rotationRate = 0.0;
        }
    }

    /**
     * Periodically samples the trajectory state at elapsed time, evaluates position and heading errors,
     * computes control velocities, and commands field-oriented drive.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        if (trajectory != null) {
            // Sample reference state along the trajectory
            State targetPathState = trajectory.sample(pathTime.seconds());

            // Current robot pose
            Pose2d currentPos = RobotContainer.odometry.getCurrentPos();

            // Calculate expected robot body heading
            double targetRobotAngle;
            if (pathTime.seconds() < trajectory.getTotalTimeSeconds())
                targetRobotAngle = startRobotAngle + (rotationRate * pathTime.seconds());
            else
                targetRobotAngle = startRobotAngle + (rotationRate * trajectory.getTotalTimeSeconds());

            // Compute X position error and PI feedback
            double error = targetPathState.poseMeters.getX() - currentPos.getX();
            x_ierror += 0.60 * error;
            if (Math.abs(error) > 0.03)
                x_ierror = 0.0;
            double dX = 8.0 * error + x_ierror;

            // Compute Y position error and PI feedback
            double y_error = targetPathState.poseMeters.getY() - currentPos.getY();
            y_ierror += 0.60 * y_error;
            if (Math.abs(y_error) > 0.03)
                y_ierror = 0.0;
            double dY = 8.0 * y_error + y_ierror;

            // Compute rotational heading feedback
            double omega = -thetaController.calculate(Utils.AngleDifferenceRads(
                    currentPos.getRotation().getRadians(), targetRobotAngle));

            // Command field-oriented motion
            RobotContainer.drivesystem.FieldDrive(dX, dY, omega);
        }
    }

    /**
     * Checks whether trajectory following has finished or timed out.
     *
     * @author gbuckholtz
     *
     * @return true if trajectory is null or elapsed time exceeds trajectory duration (+0.10s)
     */
    @Override
    public boolean isFinished() {
        return (trajectory == null || (pathTime.seconds() > (trajectory.getTotalTimeSeconds() + 0.10)));
    }

    /**
     * Stops drivetrain if trajectory endSpeed is 0.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        if (endSpeed <= 0.01) {
            RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0);
        }
    }
}