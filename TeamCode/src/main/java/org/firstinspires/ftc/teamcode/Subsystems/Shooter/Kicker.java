package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to control the kicker mechanism using continuous rotation servos.
 *
 * @author superzokabear
 */
public class Kicker extends SubsystemBase {

    // Kicker servo hardware
    private final Servo leftRampServo;
    private final Servo rightRampServo;

    // Continuous servo power: 1.0 = forward/full speed, 0.5 = stopped
    private static final double SERVO_ON = 1.0;
    private static final double SERVO_OFF = 0.5;

    // Tunable duration constant in seconds to rotate the kicker approximately 180 degrees
    public static double KICK_PAUSE_DURATION_SECONDS = 0.35;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean isKicking = false;

    /**
     * Initializes the kicker subsystem and configures continuous servos to run in opposite directions.
     *
     * @author superzokabear
     */
    public Kicker() {
        // Retrieve kicker servos from the active OpMode's hardware map
        leftRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
        rightRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

        // Servos run in opposite directions
        leftRampServo.setDirection(Servo.Direction.FORWARD);
        rightRampServo.setDirection(Servo.Direction.REVERSE);

        // Set initial state to stopped
        stop();
    }

    /**
     * Method called periodically by the command scheduler.
     * Checks if the kicker is active and stops the servos once the tuned duration has elapsed.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        if (isKicking && kickTimer.seconds() >= KICK_PAUSE_DURATION_SECONDS) {
            stop();
        }
    }

    /**
     * Starts continuous rotation on both servos in opposite directions and resets the kick timer.
     *
     * @author superzokabear
     */
    public void kick() {
        leftRampServo.setPosition(SERVO_ON);
        rightRampServo.setPosition(SERVO_ON);
        kickTimer.reset();
        isKicking = true;
    }

    /**
     * Stops continuous rotation on both servos.
     *
     * @author superzokabear
     */
    public void stop() {
        leftRampServo.setPosition(SERVO_OFF);
        rightRampServo.setPosition(SERVO_OFF);
        isKicking = false;
    }

    /**
     * Checks whether the kicker is currently in motion.
     *
     * @return true if kicker is currently rotating; false otherwise
     */
    public boolean isKicking() {
        return isKicking;
    }

    /**
     * Backward-compatible alias for checking if the kicker is running.
     *
     * @return true if kicker is running; false otherwise
     */
    public boolean rampIsRunning() {
        return isKicking;
    }
}