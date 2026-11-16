package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem for detecting game element (ball) presence and classifying its type/color
 * using a top-mounted REV Color Sensor V3 located above the shooter feed chamber.
 * <p>
 * <b>Teaching Concept - REV Color Sensor V3 in Color & Proximity Modes:</b>
 * The REV Color Sensor V3 combines an ambient light/color sensor (measuring Red, Green, Blue,
 * and Clear/Alpha channels) with an Infrared (IR) proximity photodiode.
 * In this subsystem:
 * <ul>
 *   <li><b>Proximity / Distance:</b> Used to quickly check whether a ball is seated in the chamber
 *       (e.g., within 50 mm).</li>
 *   <li><b>Color Mode:</b> Analyzes the reflected light channels or converts RGB to HSV
 *       (Hue, Saturation, Value) to classify the game element (e.g. Nectar vs. Pollen).
 *       Using HSV Hue angles makes detection resilient to varying arena lighting.</li>
 * </ul>
 *
 * @author gbuckholtz
 */
public class BallColorSensor extends SubsystemBase {

    /**
     * Enumeration representing the possible classifications of detected game elements.
     */
    public enum BallType {
        NONE,       // No ball detected in the chamber
        NECTAR,     // Nectar ball detected (larger ball size / requires wider flywheel gap)
        POLLEN,     // Pollen ball detected (smaller ball size / requires narrower flywheel gap)
        UNKNOWN     // Ball is present, but color profile does not confidently match Nectar or Pollen
    }

    // REV Color Sensor V3 hardware reference
    private RevColorSensorV3 topColorSensor;
    private boolean isSensorInitialized = false;

    // Distance below which a ball is considered physically present under the sensor (in millimeters)
    public static final double BALL_PRESENT_DISTANCE_THRESHOLD_MM = 50.0;

    // Cached state from periodic updates
    private BallType currentBallType = BallType.NONE;
    private boolean ballPresent = false;
    private double lastDistanceMm = 999.0;
    private final float[] hsvValues = new float[3];

    /**
     * Constructs the BallColorSensor subsystem and initializes the REV Color Sensor V3
     * from the active OpMode's hardware map.
     *
     * @author gbuckholtz
     */
    public BallColorSensor() {
        initializeSensor();
    }

    /**
     * Safely retrieves and initializes the sensor device from the hardware map.
     * Includes exception handling to prevent robot crashes if the sensor is unplugged or misconfigured.
     */
    private void initializeSensor() {
        try {
            if (RobotContainer.ActiveOpMode != null && RobotContainer.ActiveOpMode.hardwareMap != null) {
                topColorSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "topBallSensor");
                topColorSensor.initialize();
                isSensorInitialized = true;
            }
        } catch (Exception e) {
            isSensorInitialized = false;
            if (RobotContainer.telemetrySubsystem != null) {
                RobotContainer.telemetrySubsystem.addData("TopBallSensor Init Error", e.getMessage());
            }
        }
    }

    /**
     * Method called periodically by the FTCLib CommandScheduler.
     * Polls the sensor to update distance, color readings, and ball classification.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        if (!isSensorInitialized || topColorSensor == null) {
            // If initialization previously failed, attempt recovery
            initializeSensor();
            if (!isSensorInitialized) {
                ballPresent = false;
                currentBallType = BallType.NONE;
                return;
            }
        }

        try {
            // 1. Measure distance in mm to determine presence
            lastDistanceMm = topColorSensor.getDistance(DistanceUnit.MM);
            ballPresent = (lastDistanceMm < BALL_PRESENT_DISTANCE_THRESHOLD_MM);

            if (!ballPresent) {
                currentBallType = BallType.NONE;
            } else {
                // 2. Classify ball color when present
                currentBallType = evaluateBallColor();
            }

            // 3. Telemetry for debugging and calibration during testing
            if (RobotContainer.telemetrySubsystem != null) {
                RobotContainer.telemetrySubsystem.addData("Ball Present", ballPresent);
                RobotContainer.telemetrySubsystem.addData("Ball Type", currentBallType.name());
                RobotContainer.telemetrySubsystem.addData("Ball Dist (mm)",
                        String.format(java.util.Locale.US, "%.1f", lastDistanceMm));
            }
        } catch (Exception e) {
            // Catch I2C communication dropouts gracefully
            ballPresent = false;
            currentBallType = BallType.NONE;
        }
    }

    /**
     * Evaluates the RGB channels from the sensor and determines whether the ball is Nectar, Pollen, or Unknown.
     * <p>
     * <b>Teaching Note:</b>
     * Converting raw RGB to HSV (Hue: 0-360°, Saturation: 0-1, Value: 0-1) is generally more
     * robust than comparing raw RGB values because Hue isolates the color identity from
     * brightness changes.
     *
     * @return classified BallType
     */
    private BallType evaluateBallColor() {
        if (topColorSensor == null) return BallType.NONE;

        int r = topColorSensor.red();
        int g = topColorSensor.green();
        int b = topColorSensor.blue();

        // Convert RGB to HSV
        Color.RGBToHSV(r * 8, g * 8, b * 8, hsvValues);
        float hue = hsvValues[0];
        float sat = hsvValues[1];
        float val = hsvValues[2];

        // Example color classification logic using Hue, Saturation, and Value:
        // Teaching Tip: Checking saturation and value ensures that dark shadows or white arena lights
        // don't trigger false positives.
        if (sat > 0.2f && val > 0.1f) {
            if (hue >= 45.0f && hue <= 120.0f) {
                return BallType.NECTAR;
            } else if (hue >= 10.0f && hue < 45.0f) {
                return BallType.POLLEN;
            }
        }
        return BallType.UNKNOWN;
    }

    /**
     * Checks if a game element (ball) is currently present in the shooter feed position.
     *
     * @return true if an element is detected within the distance threshold; false otherwise
     */
    public boolean isBallPresent() {
        return ballPresent;
    }

    /**
     * Retrieves the classified type of the currently detected ball.
     *
     * @return detected {@link BallType} (NONE, NECTAR, POLLEN, or UNKNOWN)
     */
    public BallType getBallType() {
        return currentBallType;
    }

    /**
     * Checks specifically if the detected game element is Nectar.
     *
     * @return true if a ball is present and classified as Nectar; false otherwise
     */
    public boolean isNectar() {
        return ballPresent && currentBallType == BallType.NECTAR;
    }

    /**
     * Retrieves the most recent distance reading in millimeters.
     *
     * @return distance in millimeters
     */
    public double getDistanceMm() {
        return lastDistanceMm;
    }
}
