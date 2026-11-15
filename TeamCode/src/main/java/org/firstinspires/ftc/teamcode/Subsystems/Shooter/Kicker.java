package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.Locale;

/**
 * Subsystem to control the shooter kicker mechanism.
 * Uses continuous rotation servos equipped with paddles/fins to kick game elements
 * into the shooter mechanism.
 * <p>
 * <b>Teaching Concept - Distance Sensors as Virtual Rotary Limit Switches:</b>
 * Continuous rotation servos do not have built-in absolute position feedback (encoders).
 * To ensure the paddles/fins do not stop inside the feed tunnel and block incoming balls,
 * two REV Color Sensor V3 devices operate in <b>Distance Mode</b> on either side of the kicker.
 * <ul>
 *   <li>When a fin is blocking or entering the feed path, the distance reading drops below
 *       {@link #FIN_BLOCKED_DISTANCE_THRESHOLD_CM}.</li>
 *   <li>When the fin rotates fully out of the feed corridor, the measured distance exceeds
 *       {@link #FIN_CLEAR_DISTANCE_THRESHOLD_CM}, signaling that the path in is clear.</li>
 * </ul>
 *
 * @author superzokabear, gbuckholtz
 */
public class Kicker extends SubsystemBase {

    /**
     * Operational state of the kicker subsystem.
     */
    public enum KickerState {
        STOPPED,        // Servos are stopped
        KICKING,        // Servos advancing at full speed to launch the ball
        CLEARING_PATH   // Servos advancing until fins clear the feed path, then stopping
    }

    // Kicker servo hardware (servos equipped with paddles to kick game elements)
    private Servo leftRampServo;
    private Servo rightRampServo;

    // Side REV Color Sensor V3 hardware in distance mode
    private RevColorSensorV3 leftFinSensor;
    private RevColorSensorV3 rightFinSensor;
    private boolean areSensorsInitialized = false;

    // Continuous servo power: 1.0 = forward/full speed, 0.5 = stopped
    private static final double SERVO_ON = 1.0;
    private static final double SERVO_OFF = 0.5;

    // Distance calibration thresholds (in centimeters)
    // Distance beyond which the fin is rotated away and the feed path is unobstructed
    public static final double FIN_CLEAR_DISTANCE_THRESHOLD_CM = 6.0;
    // Distance when the fin passes directly in front of the side sensor
    public static final double FIN_BLOCKED_DISTANCE_THRESHOLD_CM = 3.5;

    private KickerState currentState = KickerState.STOPPED;
    private double lastLeftDistanceCm = 999.0;
    private double lastRightDistanceCm = 999.0;

    /**
     * Initializes the kicker subsystem, configures the continuous rotation paddle servos,
     * and initializes the two side distance sensors.
     *
     * @author superzokabear, gbuckholtz
     */
    public Kicker() {
        // Retrieve kicker servos from the active OpMode's hardware map
        try {
            leftRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
            rightRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

            // Servos run in opposite directions to sweep paddles synchronously
            leftRampServo.setDirection(Servo.Direction.FORWARD);
            rightRampServo.setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            if (RobotContainer.telemetrySubsystem != null) {
                RobotContainer.telemetrySubsystem.addData("Kicker Servo Init Error", e.getMessage());
            }
        }

        // Initialize the two side REV Color Sensor V3 devices in distance mode
        initializeSensors();

        // Set initial state to stopped
        stop();
    }

    /**
     * Safely retrieves and initializes the side distance sensors from the hardware map.
     */
    private void initializeSensors() {
        try {
            if (RobotContainer.ActiveOpMode != null && RobotContainer.ActiveOpMode.hardwareMap != null) {
                leftFinSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "leftFinSensor");
                rightFinSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "rightFinSensor");

                if (leftFinSensor != null) leftFinSensor.initialize();
                if (rightFinSensor != null) rightFinSensor.initialize();
                areSensorsInitialized = true;
            }
        } catch (Exception e) {
            areSensorsInitialized = false;
            if (RobotContainer.telemetrySubsystem != null) {
                RobotContainer.telemetrySubsystem.addData("Kicker Fin Sensors Init Error", e.getMessage());
            }
        }
    }

    /**
     * Method called periodically by the command scheduler.
     * Monitors the side distance sensors. If in {@link KickerState#CLEARING_PATH},
     * automatically stops the servos once both fins rotate clear of the feed corridor.
     *
     * @author superzokabear, gbuckholtz
     */
    @Override
    public void periodic() {
        updateSensorReadings();

        // Automatic path clearing logic:
        // Once the feed path is clear on both sides, stop the paddles
        if (currentState == KickerState.CLEARING_PATH) {
            if (isFeedPathClear()) {
                stop();
            }
        }

        // Telemetry readout for tuning and sensor verification
        if (RobotContainer.telemetrySubsystem != null) {
            RobotContainer.telemetrySubsystem.addData("Kicker State", currentState.name());
            RobotContainer.telemetrySubsystem.addData("Feed Clear", isFeedPathClear());
            RobotContainer.telemetrySubsystem.addData("Left Fin (cm)",
                    String.format(Locale.US, "%.1f", lastLeftDistanceCm));
            RobotContainer.telemetrySubsystem.addData("Right Fin (cm)",
                    String.format(Locale.US, "%.1f", lastRightDistanceCm));
        }
    }

    /**
     * Polls the two side sensors to read current distance in centimeters.
     */
    private void updateSensorReadings() {
        if (!areSensorsInitialized) {
            initializeSensors();
            if (!areSensorsInitialized) return;
        }

        try {
            if (leftFinSensor != null) {
                lastLeftDistanceCm = leftFinSensor.getDistance(DistanceUnit.CM);
            }
            if (rightFinSensor != null) {
                lastRightDistanceCm = rightFinSensor.getDistance(DistanceUnit.CM);
            }
        } catch (Exception e) {
            // Gracefully catch intermittent I2C communication exceptions
        }
    }

    /**
     * Checks if the left kicker fin is clear of the feed corridor.
     *
     * @return true if left sensor distance is greater than the clear threshold; false if fin is detected
     */
    public boolean isLeftFinClear() {
        return lastLeftDistanceCm >= FIN_CLEAR_DISTANCE_THRESHOLD_CM;
    }

    /**
     * Checks if the right kicker fin is clear of the feed corridor.
     *
     * @return true if right sensor distance is greater than the clear threshold; false if fin is detected
     */
    public boolean isRightFinClear() {
        return lastRightDistanceCm >= FIN_CLEAR_DISTANCE_THRESHOLD_CM;
    }

    /**
     * Determines whether the feed path into the shooter is clear of both kicker fins.
     *
     * @return true when both left and right fins are rotated out of the path; false otherwise
     */
    public boolean isFeedPathClear() {
        return isLeftFinClear() && isRightFinClear();
    }

    /**
     * Starts continuous rotation on both kicker servos to swing the paddles and kick the game element.
     *
     * @author superzokabear
     */
    public void kick() {
        advanceToShoot();
    }

    /**
     * Advances the kicker paddles at full speed to propel the ball into the spinning flywheels.
     *
     * @author gbuckholtz
     */
    public void advanceToShoot() {
        if (leftRampServo != null) leftRampServo.setPosition(SERVO_ON);
        if (rightRampServo != null) rightRampServo.setPosition(SERVO_ON);
        currentState = KickerState.KICKING;
    }

    /**
     * Rotates the paddles until the side distance sensors detect that the fins
     * are clear of the feed path, at which point the paddles stop automatically.
     *
     * @author gbuckholtz
     */
    public void clearFeedPath() {
        if (isFeedPathClear()) {
            stop();
            return;
        }
        if (leftRampServo != null) leftRampServo.setPosition(SERVO_ON);
        if (rightRampServo != null) rightRampServo.setPosition(SERVO_ON);
        currentState = KickerState.CLEARING_PATH;
    }

    /**
     * Stops continuous rotation on both kicker paddle servos.
     *
     * @author superzokabear
     */
    public void stop() {
        if (leftRampServo != null) leftRampServo.setPosition(SERVO_OFF);
        if (rightRampServo != null) rightRampServo.setPosition(SERVO_OFF);
        currentState = KickerState.STOPPED;
    }

    /**
     * Checks whether the kicker paddles are currently in motion.
     *
     * @return true if kicker paddles are currently rotating; false otherwise
     */
    public boolean isKicking() {
        return currentState == KickerState.KICKING;
    }

    /**
     * Returns the current operational state of the kicker.
     *
     * @return {@link KickerState}
     */
    public KickerState getCurrentState() {
        return currentState;
    }

    /**
     * Gets the latest measured distance for the left fin sensor in centimeters.
     *
     * @return distance in centimeters
     */
    public double getLeftDistanceCm() {
        return lastLeftDistanceCm;
    }

    /**
     * Gets the latest measured distance for the right fin sensor in centimeters.
     *
     * @return distance in centimeters
     */
    public double getRightDistanceCm() {
        return lastRightDistanceCm;
    }
}
