package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem that manages dead-wheel odometry pods via the goBILDA Pinpoint Odometry Computer.
 * Tracks field position (X, Y) and heading using dedicated high-speed sensor fusion.
 *
 * @author gbuckholtz
 */
public class PinpointOdometry extends SubsystemBase {

    // GoBilda Pinpoint hardware driver
    private GoBildaPinpointDriver pinpointDriver;

    /**
     * Initializes the Pinpoint Odometry subsystem, configures pod encoder directions,
     * encoder resolution, sensor physical offsets, and resets position and IMU heading.
     *
     * @author gbuckholtz
     */
    public PinpointOdometry() {
        // Retrieve the Pinpoint driver from the active OpMode's hardware map
        pinpointDriver = RobotContainer.ActiveOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "PinpointDriver");

        // Set encoder directions for forward and reversed tracking pods
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Configure encoder resolution corresponding to goBILDA 4-bar swingarm pods
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Configure X and Y pod offsets from the robot center of rotation in millimeters
        pinpointDriver.setOffsets(+44.0, 108.0, DistanceUnit.MM);

        // Reset pinpoint driver position and integrated IMU heading
        pinpointDriver.resetPosAndIMU();

        // Pause briefly for the Pinpoint computer to complete initialization and calibration
        RobotContainer.ActiveOpMode.sleep(300);
    }

    /**
     * Periodic method called periodically by the command scheduler to poll the Pinpoint driver.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Poll and update internal odometry tracking data from the Pinpoint device
        pinpointDriver.update();
    }

    /**
     * Resets the robot odometry position and integrated IMU heading to zero.
     *
     * @author gbuckholtz
     */
    public void reset() {
        // Reset device position and IMU heading
        pinpointDriver.resetPosAndIMU();
    }

    /**
     * Overrides and sets the robot's current field position and heading to the specified Pose2d.
     *
     * @author gbuckholtz
     *
     * @param pose the desired Pose2d to set the robot position (in meters) and heading (in radians)
     */
    public void SetPose(Pose2d pose) {
        // Convert FTCLib Pose2d into SDK Pose2D object in meters and radians
        Pose2D newPose = new Pose2D(DistanceUnit.METER, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());

        // Update heading and position on the Pinpoint computer
        pinpointDriver.setHeading(pose.getHeading(), AngleUnit.RADIANS);
        pinpointDriver.setPosition(newPose);
    }

    /**
     * Retrieves the current field position and heading of the robot.
     *
     * @author gbuckholtz
     *
     * @return Pose2d containing X and Y positions in meters and heading in radians
     */
    public Pose2d GetPose() {
        // Read position in meters and heading in radians from Pinpoint device
        Pose2d pose = new Pose2d(
                pinpointDriver.getPosX(DistanceUnit.METER),
                pinpointDriver.getPosY(DistanceUnit.METER),
                new Rotation2d(pinpointDriver.getHeading(AngleUnit.RADIANS)));

        return pose;
    }
}