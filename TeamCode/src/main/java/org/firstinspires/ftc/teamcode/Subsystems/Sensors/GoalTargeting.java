package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem responsible for calculating robot distance to alliance goals,
 * verifying valid shooting zones, and automatically adjusting the shooter flywheel
 * speed and hood tilt angle.
 *
 * @author knutt5
 */
public class GoalTargeting extends SubsystemBase {

    // Alliance goal coordinates on the field in meters
    Translation2d redGoalNear = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalNear = new Translation2d(-1.63, -1.55);
    Translation2d redGoalFar = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalFar = new Translation2d(-1.63, -1.55);

    // Enable/disable auto target mode
    private boolean AutoTargetingEnable;

    // Counter keeps track of how many times camera does not detect target
    int MissedCameraTargetCount;

    /**
     * Initializes the goal targeting subsystem with auto-targeting enabled by default.
     *
     * @author knutt5
     */
    public GoalTargeting() {
        // By default, enable auto targeting
        AutoTargetingEnable = true;
        MissedCameraTargetCount = 0;
    }

    /**
     * Method called periodically by the command scheduler to determine distance to the goal
     * and adjust shooter hood angle and flywheel speed accordingly.
     *
     * @author knutt5
     */
    @Override
    public void periodic() {
        if (AutoTargetingEnable) {
            // Determine if target is visible via camera, otherwise fallback to odometry
            LLResult results = RobotContainer.limeLight.getLimeLightResults();
            boolean CameraHasValidTarget = false;
            double CameraTargetXAngle = 0.0;
            double TargetDistance = 0.0;

            // Increment missed camera target counter
            MissedCameraTargetCount++;

            // Evaluate Limelight fiducial results
            if (results != null && results.isValid() && results.getFiducialResults() != null &&
                    !results.getFiducialResults().isEmpty() && results.getStaleness() < 100) {
                for (int i = 0; i < results.getFiducialResults().size(); ++i) {
                    if ((RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 24) ||
                            (!RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 20)) {
                        // Valid target hit found
                        CameraHasValidTarget = true;
                        MissedCameraTargetCount = 0;
                        CameraTargetXAngle = results.getFiducialResults().get(i).getTargetXDegrees();
                        Position vector = results.getFiducialResults().get(i).getTargetPoseRobotSpace().getPosition();
                        TargetDistance = Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
                    }
                }
            }

            // Calculate distance to goal using odometry
            TargetDistance = GetDistanceToGoal();

            // Display distance to telemetry
            RobotContainer.telemetrySubsystem.addData("distance", TargetDistance, true);

            // Set shooter hood position and flywheel RPM based on distance
            RobotContainer.hoodtilt.SetHoodPosition(CalculateHoodAngle(TargetDistance));
            RobotContainer.shooter.SetFlywheelSpeed(CalculateSpeed(TargetDistance));
        }
    }

    /**
     * Normalizes an angle in degrees to the [-180, 180) range.
     *
     * @author knutt5
     *
     * @param degrees angle in degrees to normalize
     * @return normalized angle in degrees between -180 and 180
     */
    private double normalizeAngle(double degrees) {
        // Wrap angle into [-180, 180)
        return ((degrees % 360.0) + 360.0) % 360.0 - 180.0;
    }

    /**
     * Enables or disables automatic shooter aiming and speed adjustment.
     * Turning auto-targeting off will spin down the shooter flywheel.
     *
     * @author knutt5
     *
     * @param enable true to enable auto-targeting, false to disable
     */
    public void EnableAutoTargeting(boolean enable) {
        AutoTargetingEnable = enable;

        // If switching off enable, stop the flywheel
        if (!enable) {
            RobotContainer.shooter.SetFlywheelSpeed(0.0);
        }
    }

    /**
     * Checks if automatic targeting is currently active.
     *
     * @author knutt5
     *
     * @return true if auto-targeting is enabled; false otherwise
     */
    public boolean isAutoTargetingEnabled() {
        return AutoTargetingEnable;
    }

    /**
     * Enumeration of allowable shooting sides.
     */
    public enum ShootSide {
        LEFT, RIGHT, NONE, BOTH
    }

    /**
     * Calculates the Euclidean distance from the robot's current odometry position to the active alliance goal.
     *
     * @author kaitlyn
     *
     * @return distance to the target goal in meters
     */
    public double GetDistanceToGoal() {
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();
        Translation2d goalPose;

        // Select alliance goal coordinates
        if (RobotContainer.isRedAlliance()) {
            if (currentPos.getX() <= 1.90)
                goalPose = redGoalNear;
            else
                goalPose = redGoalFar;
        } else {
            if (currentPos.getX() <= 1.90)
                goalPose = blueGoalNear;
            else
                goalPose = blueGoalFar;
        }

        // Calculate delta X and delta Y
        double x = goalPose.getX() - currentPos.getX();
        double y = goalPose.getY() - currentPos.getY();

        // Return Euclidean distance in meters
        return Math.sqrt((x * x) + (y * y));
    }

    /**
     * Calculates the required shooter flywheel rotational speed (RPM) for a given distance to the goal.
     *
     * @author kaitlyn
     *
     * @param distance distance to goal in meters
     * @return required flywheel speed in RPM
     */
    public double CalculateSpeed(double distance) {
        // Empirically derived linear speed curve
        return 426.05 * distance + 1606;
    }

    /**
     * Calculates the required shooter hood tilt servo position for a given distance to the goal.
     *
     * @author superzokabear
     *
     * @param distance distance to goal in meters
     * @return required hood servo position
     */
    public double CalculateHoodAngle(double distance) {
        // Empirically derived quadratic hood position curve
        return -0.0096 * distance * distance + 0.0839 * distance - 0.0655;
    }

    /**
     * Retrieves the coordinates of the primary target goal for the active alliance.
     *
     * @author superzokabear
     *
     * @return Translation2d representing the field coordinates of the active goal in meters
     */
    public Translation2d GetShotTaget() {
        // Return near goal based on current alliance
        if (RobotContainer.isRedAlliance()) {
            return redGoalNear;
        } else {
            return blueGoalNear;
        }
    }

    /**
     * Determines whether the robot's current odometry position is within an allowable shooting zone on the field.
     *
     * @author knutt5
     *
     * @return true if the robot is located inside an allowed shooting zone, false otherwise
     */
    public boolean IsRobotInAllowableShotZone() {
        // Read current robot field position from odometry
        Pose2d pos = RobotContainer.odometry.getCurrentPos();
        double x = pos.getX();
        double y = pos.getY();

        // 15cm margin around shooting zone
        double margin = 0.15;

        // Check if robot is in triangular region near goals (>80cm from corner) or rear triangular region
        if ((x < (0.0 + margin) &&
                Math.abs(x) > Math.abs(y) - margin &&
                Math.abs(y) < 1.0) ||
                (x >= (1.1 - margin) &&
                (x - 1.1) > Math.abs(y) - margin)) {
            return true;
        } else {
            return false;
        }
    }
}