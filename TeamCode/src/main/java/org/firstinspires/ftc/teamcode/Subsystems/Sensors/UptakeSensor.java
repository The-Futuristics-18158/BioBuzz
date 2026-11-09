package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem that uses a REV Color Sensor V3 on the intake ramp to detect
 * when a game artifact is present and ready for uptake or shooting.
 *
 * @author Blackthrush
 */
public class UptakeSensor extends SubsystemBase {

    // REV Color Sensor V3 hardware
    private RevColorSensorV3 uptakeSensor;
    private boolean isUptakeSensorInitialized = true;

    /**
     * Initializes the uptake sensor from the hardware map.
     *
     * @author Blackthrush
     */
    public UptakeSensor() {
        // Retrieve and initialize the color/distance sensor from the hardware map
        uptakeSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "uptakeSensor");
        uptakeSensor.initialize();
    }

    /**
     * Method called periodically by the scheduler.
     *
     * @author Blackthrush
     */
    @Override
    public void periodic() {
        // Periodic sensor monitoring if needed
    }

    /**
     * Uses the distance sensor to determine if an artifact is present on the intake ramp.
     * Includes error handling and automatic sensor re-initialization if a communication error occurs.
     *
     * @author Blackthrush
     *
     * @return true if an artifact is detected within 5 cm, false otherwise
     */
    public boolean isUpakeArtifactPresent() {
        try {
            // Read distance from the sensor in centimeters
            double distance = uptakeSensor.getDistance(DistanceUnit.CM);

            // An artifact is present if the measured distance is less than 5.0 cm
            return (distance < 5.0);
        } catch (Exception e) {
            // Log the error to telemetry
            RobotContainer.telemetrySubsystem.addData("uptake sensor Error", e.getMessage());
            RobotContainer.telemetrySubsystem.update();

            // Attempt recovery if first failure
            if (isUptakeSensorInitialized) {
                isUptakeSensorInitialized = false;
                try {
                    // Re-fetch and re-initialize the sensor device
                    uptakeSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "uptakeSensor");
                    uptakeSensor.initialize();
                    isUptakeSensorInitialized = true;
                } catch (Exception ex) {
                    // Recovery failed
                    RobotContainer.telemetrySubsystem.addData("uptake sensor recovery failed", ex.getMessage());
                    RobotContainer.telemetrySubsystem.update();
                }
            }

            // Return false if unable to read distance reliably
            return false;
        }
    }
}