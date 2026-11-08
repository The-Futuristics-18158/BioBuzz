package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Robot gyroscope subsystem wrapping the internal Control Hub IMU.
 * Provides heading (Yaw) and tilt (Roll) measurement with support for angle offsets and zeroing.
 *
 * @author gbuckholtz
 */
public class Gyro extends SubsystemBase {

    // Robot gyroscope IMU hardware
    private IMU gyro;

    // Current filtered angles in degrees
    private double YawAngle;
    private double RollAngle;

    // Gyro offsets in degrees
    private double YawAngleOffset;
    private double RollAngleOffset;

    /**
     * Initializes the gyroscope subsystem, configures hub orientation, resets yaw,
     * and zeroes initial roll angle offset.
     *
     * @author gbuckholtz
     */
    public Gyro() {
        // Reset yaw offset to zero
        YawAngleOffset = 0.0;

        // Retrieve the IMU hardware device from the active OpMode's hardware map
        gyro = RobotContainer.ActiveOpMode.hardwareMap.get(IMU.class, "imu");

        // Initialize orientation: logo facing backward, USB facing up
        gyro.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reset yaw angle
        gyro.resetYaw();

        // Calculate initial roll offset so roll starts at zero
        RollAngleOffset = -gyro.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    /**
     * Method called periodically by the command scheduler to update current yaw and roll angles.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Read raw yaw and apply calibration offset
        YawAngle = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + YawAngleOffset;

        // Read raw roll and apply calibration offset
        RollAngle = gyro.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) + RollAngleOffset;
    }

    /**
     * Retrieves the current robot yaw (heading) angle in degrees.
     *
     * @author gbuckholtz
     *
     * @return current yaw angle in degrees between -180 and 180
     */
    public double getYawAngle() {
        // Return calibrated yaw angle
        return YawAngle;
    }

    /**
     * Retrieves the current robot roll angle in degrees.
     *
     * @author gbuckholtz
     *
     * @return current roll angle in degrees between -180 and 180
     */
    public double getRollAngle() {
        // Return calibrated roll angle
        return RollAngle;
    }

    /**
     * Resets the gyro yaw angle to 0.0 degrees.
     *
     * @author gbuckholtz
     */
    public void resetYawAngle() {
        // Zero the current heading angle
        setYawAngle(0.0);
    }

    /**
     * Sets the gyro yaw angle to a specified angle in degrees by updating the offset.
     *
     * @author gbuckholtz
     *
     * @param angle the new heading angle in degrees
     */
    public void setYawAngle(double angle) {
        // Adjust the offset difference between current yaw and requested angle
        YawAngleOffset -= getYawAngle() - angle;
    }
}