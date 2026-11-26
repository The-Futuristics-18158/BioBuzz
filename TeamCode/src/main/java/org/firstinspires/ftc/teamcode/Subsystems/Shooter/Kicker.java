package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    private final Servo leftKickerServo;
    private final Servo rightKickerServo;


    private DistanceSensor uptakeSensor_DistanceSensor;



    // Continuous servo power: 1.0 = forward/full speed, 0.5 = stopped
    private static final double KICKER_UP = 0.5;
    private static final double KICKER_DOWN = 0.0;

    // Tunable duration constant in seconds to rotate the kicker approximately 180 degrees
    public static double KICK_PAUSE_DURATION_SECONDS = 0.35;

    private ElapsedTime kickTimer = new ElapsedTime();
    private boolean isKicking = false;

    /**
     * Initializes the kicker subsystem and configures continuous servos to run in opposite directions.
     *
     * @author superzokabear
     */
    public Kicker() {
        // Retrieve kicker servos from the active OpMode's hardware map
        leftKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
        rightKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

        // Servos run in opposite directions
        leftKickerServo.setDirection(Servo.Direction.FORWARD);
        rightKickerServo.setDirection(Servo.Direction.REVERSE);

        leftKickerServo.setPosition(KICKER_DOWN);
        rightKickerServo.setPosition(KICKER_DOWN);
        // Set initial state to stopped
        //stop();
    }

    /**
     * Method called periodically by the command scheduler.
     * Checks if the kicker is active and stops the servos once the tuned duration has elapsed.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        if(kickTimer.milliseconds()>1000.0) {
            park();
        }
        RobotContainer.telemetrySubsystem.addData("Timer in MS", kickTimer.milliseconds());
    }

    /**
     * Starts continuous rotation on both servos in opposite directions and resets the kick timer.
     *
     * @author superzokabear
     */
    public void kick() {
        kickTimer = new ElapsedTime();
        leftKickerServo.setPosition(KICKER_UP);
        rightKickerServo.setPosition(KICKER_UP);
    }

    public void park() {
        leftKickerServo.setPosition(KICKER_DOWN);
        rightKickerServo.setPosition(KICKER_DOWN);

        RobotContainer.telemetrySubsystem.addData("Kicker Status Down", 10);
    }

    /**
     * Stops continuous rotation on both servos.
     *
     * @author superzokabear
     */
    public void stop() {
        isKicking=false;
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