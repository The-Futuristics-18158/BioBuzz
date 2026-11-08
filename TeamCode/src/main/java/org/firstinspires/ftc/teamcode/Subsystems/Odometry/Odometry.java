package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Main robot odometry subsystem.
 * Fuses dead-wheel Pinpoint odometry tracking with Limelight MegaTag2 AprilTag vision updates
 * when driving at low speeds during TeleOp to correct for long-term drift.
 *
 * @author gbuckholtz
 */
public class Odometry extends SubsystemBase {

    // Stored robot position used to persist robot pose across OpModes
    private static Pose2d StoredRobotPose = new Pose2d(0, 0, new Rotation2d(0));

    // Current robot position in meters
    private Pose2d CurrentPose;

    /**
     * Initializes the odometry subsystem and restores the initial pose from previously stored values.
     *
     * @author gbuckholtz
     */
    public Odometry() {
        // Initialize field position from stored value (i.e. previous OpMode)
        setCurrentPos(StoredRobotPose);
    }

    /**
     * Method called periodically by the command scheduler to update the robot pose from Pinpoint pods,
     * filter in vision updates from AprilTags when stationary/low-speed, and persist pose.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Retrieve current dead-wheel odometry pose
        CurrentPose = RobotContainer.odometryPod.GetPose();

        // Query Limelight results for AprilTag Megatag2 localization
        LLResult results = RobotContainer.limeLight.getLimeLightResults();
        boolean CameraHasValidTarget = false;
        Position RobotPosition = new Position();

        // Check if camera has valid, fresh fiducial detections for the alliance target
        if (results != null && results.isValid() && results.getFiducialResults() != null &&
                !results.getFiducialResults().isEmpty() && results.getStaleness() < 100) {
            for (int i = 0; i < results.getFiducialResults().size(); ++i) {
                if ((RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 24) ||
                        (!RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 20)) {
                    // Valid AprilTag detection confirmed
                    CameraHasValidTarget = true;
                    RobotPosition = results.getBotpose().getPosition();
                }
            }
        }

        // Only fuse AprilTag pose updates in TeleOp mode when moving slowly (< 0.1 m/s)
        if (RobotContainer.GetCurrentMode() == RobotContainer.Modes.TeleOp && CameraHasValidTarget) {
            double LLpose_x = RobotPosition.x;
            double LLpose_y = RobotPosition.y;

            // Notify driver station that odometry adjustment is active
            RobotContainer.telemetrySubsystem.addLine("Adjusting Odom(AT)", true);

            MecanumDriveWheelSpeeds speed = RobotContainer.drivesystem.GetWheelSpeeds();

            // Verify Limelight position is on field and robot speed is sufficiently low
            if (LLpose_x > -1.8 && LLpose_x < 1.8 &&
                    LLpose_y > -1.8 && LLpose_y < 1.8 &&
                    speed.rearRightMetersPerSecond > -0.1 &&
                    speed.rearRightMetersPerSecond < 0.1 &&
                    speed.rearLeftMetersPerSecond > -0.1 &&
                    speed.rearLeftMetersPerSecond < 0.1 &&
                    speed.frontRightMetersPerSecond > -0.1 &&
                    speed.frontRightMetersPerSecond < 0.1 &&
                    speed.frontLeftMetersPerSecond > -0.1 &&
                    speed.frontLeftMetersPerSecond < 0.1) {
                Rotation2d gyro = new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle()));

                // Blend camera position with odometry using low-pass complementary filter
                double newpose_x = 0.99 * CurrentPose.getX() + 0.01 * LLpose_x;
                double newpose_y = 0.99 * CurrentPose.getY() + 0.01 * LLpose_y;

                // Construct new filtered pose
                CurrentPose = new Pose2d(newpose_x, newpose_y, gyro);

                // Update dead-wheel tracking pod position with filtered pose
                RobotContainer.odometry.setCurrentPos(CurrentPose);
            }
        }

        // Save position to static store so it persists across OpMode transitions
        if (!RobotContainer.ActiveOpMode.isStopRequested()) {
            StoredRobotPose = new Pose2d(CurrentPose.getX(), CurrentPose.getY(), CurrentPose.getRotation());
        }
    }

    /**
     * Retrieves the current robot field position and heading.
     *
     * @author gbuckholtz
     *
     * @return Pose2d containing robot position in meters and rotation
     */
    public Pose2d getCurrentPos() {
        // Return a copy of the current robot pose
        return new Pose2d(CurrentPose.getX(), CurrentPose.getY(), CurrentPose.getRotation());
    }

    /**
     * Sets the robot's current position to the provided Pose2d.
     * Updates internal tracking, Pinpoint odometry pods, and gyro heading offset.
     *
     * @author gbuckholtz
     *
     * @param pos Pose2d representing robot position in meters and rotation
     */
    public void setCurrentPos(Pose2d pos) {
        // Update local current pose
        CurrentPose = pos;

        // Update dead-wheel tracking pods
        RobotContainer.odometryPod.SetPose(pos);

        // Update gyro heading to match pose heading
        RobotContainer.gyro.setYawAngle(Math.toDegrees(pos.getHeading()));
    }

    /**
     * Resets the robot's current position to field origin (0, 0, 0 degrees).
     *
     * @author gbuckholtz
     */
    public void resetCurrentPos() {
        // Reset pose to origin
        setCurrentPos(new Pose2d(0, 0, new Rotation2d(0)));
    }
}