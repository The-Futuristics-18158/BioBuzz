package org.firstinspires.ftc.teamcode.Subsystems.Sensors;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author knutt5
 */
public class GoalTargeting extends SubsystemBase {

    Translation2d redGoalNear = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalNear = new Translation2d (-1.63, -1.55);
    Translation2d redGoalFar = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalFar = new Translation2d(-1.63, -1.55);

    // enable/disable auto target mode
    private boolean AutoTargetingEnable;

    // counter keeps track of how many times camera does not detect target
    int MissedCameraTargetCount;

    public GoalTargeting()
    {
        // by default, enable auto targeting
        AutoTargetingEnable = true;
        MissedCameraTargetCount = 0;
    }



    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {


        if (AutoTargetingEnable) {

            // determine if we have target in camera, otherwise use odometry
            LLResult results = RobotContainer.limeLight.getLimeLightResults();
            // do we have valid hits in camera
            boolean CameraHasValidTarget = false;
            double CameraTargetXAngle = 0.0;
            double TargetDistance = 0.0;
            // assume camera has missed target unless proven otherwise
            MissedCameraTargetCount++;

            // determine if camera has a valid target
            if (results != null && results.isValid() && results.getFiducialResults() != null &&
                    !results.getFiducialResults().isEmpty() && results.getStaleness() < 100)
                for (int i = 0; i < results.getFiducialResults().size(); ++i)
                    if ((RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 24) ||
                            (!RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 20)) {
                        // we have a hit. Record xangle and distance to the valid target
                        CameraHasValidTarget = true;
                        MissedCameraTargetCount = 0;
                        CameraTargetXAngle = results.getFiducialResults().get(i).getTargetXDegrees();
                        Position vector = results.getFiducialResults().get(i).getTargetPoseRobotSpace().getPosition();
                        TargetDistance = Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
                    }

            // ---------- Hood Angle and Flywheel Speed ----------

            //if (!CameraHasValidTarget)
            // camera does not have target, use odometry to set determine distance to target
            TargetDistance = GetDistanceToGoal();

            //if (!CameraHasValidTarget)
            RobotContainer.telemetrySubsystem.addData("distance", TargetDistance, true);
            //else
            //    RobotContainer.telemetrySubsystem.addData("distance(AT)", TargetDistance, true);

            RobotContainer.hoodtilt.SetHoodPosition(CalculateHoodAngle(TargetDistance));
            RobotContainer.shooter.SetFlywheelSpeed(CalculateSpeed(TargetDistance));


        }
    }

    private double normalizeAngle(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0 - 180.0;
    }

    /**
     * Enables/Disables auto targeting
     */
    public void EnableAutoTargeting(boolean enable)
    {
        AutoTargetingEnable = enable;
        // if switching off enable, turn off shooter
        if (!enable)
            RobotContainer.shooter.SetFlywheelSpeed(0.0);
    }

    public boolean isAutoTargetingEnabled()
    { return AutoTargetingEnable;}


    /* ---------- Left/Right Shoot Solutions ---------- */

    public enum ShootSide {
        LEFT, RIGHT, NONE, BOTH
    }


    /* ---------- Shoot Distance/Speed Calcs ---------- */

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public double GetDistanceToGoal (){
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();
        Translation2d goalPose;
        if(RobotContainer.isRedAlliance()){
            if (currentPos.getX() <=1.90)
                goalPose = redGoalNear;
            else
                goalPose = redGoalFar;
        }else{
            if (currentPos.getX() <=1.90)
                goalPose = blueGoalNear;
            else
                goalPose = blueGoalFar;
        }

        double x = goalPose.getX() - currentPos.getX();
        double y = goalPose.getY() - currentPos.getY();

        return Math.sqrt((x*x) + (y*y));
    }


    /**add description here
     *
     * @author kaitlyn
     *
     * @return Returns the speed of the flywheel
     */
    public double CalculateSpeed(double distance){

        // equation from June 04/2026 early PM 8:40pm
        return 426.05 * distance + 1606;
    }

    /**add description here
     *
     * @author superzokabear
     *
     * @return what does this return?
     */
    public double CalculateHoodAngle(double distance){

        // equation from June 04/2026 early PM 8:40pm
        return -0.0096 * distance * distance + 0.0839 * distance - 0.0655;
    }


    /**add description here
     *
     * @author superzokabear
     */
    public Translation2d GetShotTaget(){
        if(RobotContainer.isRedAlliance()){
            return redGoalNear;
        }else{
            return blueGoalNear;
        }
    }

    /** Function returns true if robot is in an allowable shooting zone
     * @return true if in zone, false if not
     */
    public boolean IsRobotInAllowableShotZone()
    {
        // get current robot position from odometry
        Pose2d pos = RobotContainer.odometry.getCurrentPos();
        double x = pos.getX();
        double y = pos.getY();

        // 15cm margin around shooting zone - robot only has to be 'over the line'
        double margin = 0.15;
//        if (x<(0.0 + margin) &&
//
//                // AND robot in front triangle (evaluated on per-side basis)
//                ((y>=0.0&& x<(-y + margin)) || (y<0.0 &&x<(y+margin))) &&
//
//                // AND not too close to target
//                //(Math.abs(x) <1.05 && Math.abs(y) < 1.05))
//            return true;
//        else if (x >= (1.1-margin) && (x-1.1)> (Math.abs(y)-margin))
//            return true;
//        else
//            return false;

        // is robot in triangular region near goals, but further than ~80cm from corner
        if ((x < (0.0+margin) &&
                Math.abs(x) > Math.abs(y)-margin &&

                // AND not too close to target
                 Math.abs(y) < 1.0) ||

                // OR robot is in rear triangular region
                (x >= (1.1-margin) &&
                (x-1.1)> Math.abs(y)-margin))
            return true;
        else
            return false;
    }

}