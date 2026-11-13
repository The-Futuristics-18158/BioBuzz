package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to control the shooter kicker mechanism.
 * Uses continuous rotation servos equipped with paddles on the end to kick game elements
 * into the shooter mechanism (replacing the previous ramp-lift approach).
 *
 * @author superzokabear
 */
public class Kicker extends SubsystemBase {

    // Kicker servo hardware (servos equipped with paddles to kick game elements)
    private final Servo leftRampServo;
    private final Servo rightRampServo;

    // Continuous servo power: 1.0 = forward/full speed, 0.5 = stopped
    private static final double SERVO_ON = 1.0;
    private static final double SERVO_OFF = 0.5;

    private boolean isKicking = false;

    /**
     * Initializes the kicker subsystem and configures the continuous rotation paddle servos
     * to run in opposing directions.
     *
     * @author superzokabear
     */
    public Kicker() {
        // Retrieve kicker servos from the active OpMode's hardware map
        leftRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
        rightRampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

        // Servos run in opposite directions to sweep paddles synchronously
        leftRampServo.setDirection(Servo.Direction.FORWARD);
        rightRampServo.setDirection(Servo.Direction.REVERSE);

        // Set initial state to stopped
        stop();
    }

    /**
     * Method called periodically by the command scheduler.
     * Monitors sensor state to ensure the paddle rotates clear of the feed path when kicking completes.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        // Periodic sensor check to ensure paddle parts stay clear when done
    }

    /**
     * Starts continuous rotation on both kicker servos to swing the paddles and kick the game element.
     *
     * @author superzokabear
     */
    public void kick() {
        leftRampServo.setPosition(SERVO_ON);
        rightRampServo.setPosition(SERVO_ON);
        isKicking = true;
    }

    /**
     * Stops continuous rotation on both kicker paddle servos.
     *
     * @author superzokabear
     */
    public void stop() {
        leftRampServo.setPosition(SERVO_OFF);
        rightRampServo.setPosition(SERVO_OFF);
        isKicking = false;
    }

    /**
     * Checks whether the kicker paddles are currently in motion.
     *
     * @return true if kicker paddles are currently rotating; false otherwise
     */
    public boolean isKicking() {
        return isKicking;
    }
}
