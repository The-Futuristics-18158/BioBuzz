package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private ColorSensor leftKickerSensor;
    private ColorSensor rightKickerSensor;
    private DistanceSensor uptakeSensor_DistanceSensor;



    // Continuous servo power: 1.0 = forward/full speed, 0.5 = stopped
    private static final double SERVO_ON = 0.5;
    private static final double SERVO_OFF = 0.0;

    // Tunable duration constant in seconds to rotate the kicker approximately 180 degrees
    public static double KICK_PAUSE_DURATION_SECONDS = 0.35;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean isKicking = false;

    private double leftKickerDistance;
    private double rightKickerDistance;
    private double blockedDistance=999.0;//in mm, was 30mm
    private boolean leftKickerParked;
    private boolean rightKickerParked;
    private boolean areKickersParked;

    /**
     * Initializes the kicker subsystem and configures continuous servos to run in opposite directions.
     *
     * @author superzokabear
     */
    public Kicker() {
        // Retrieve kicker servos from the active OpMode's hardware map
        leftKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
        rightKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

        leftKickerSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftKickerSensor");
        rightKickerSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightKickerSensor");
        // Servos run in opposite directions
        leftKickerServo.setDirection(Servo.Direction.FORWARD);
        rightKickerServo.setDirection(Servo.Direction.REVERSE);

        ((NormalizedColorSensor) leftKickerSensor).setGain(1);
        ((NormalizedColorSensor) rightKickerSensor).setGain(1);

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
        leftKickerDistance=((DistanceSensor) leftKickerSensor).getDistance(DistanceUnit.MM);
        rightKickerDistance=((DistanceSensor) rightKickerSensor).getDistance(DistanceUnit.MM);
        RobotContainer.telemetrySubsystem.addData("Left Kicker Distance",leftKickerDistance);
        RobotContainer.telemetrySubsystem.addData("Right Kicker Distance",rightKickerDistance);
        isLeftKickerParked();
        isRightKickerParked();
        parkKickers();
    }

    /**
     * Starts continuous rotation on both servos in opposite directions and resets the kick timer.
     *
     * @author superzokabear
     */
    public void kick() {
        leftKickerServo.setPosition(SERVO_ON);
        rightKickerServo.setPosition(SERVO_ON);
        isKicking = true;
        leftKickerParked=false;
        rightKickerParked=false;


    }

    public void parkKickers() {
        if(!isKicking) {
            if(!leftKickerParked) {
                leftKickerServo.setPosition(SERVO_ON);
            } else {
                leftKickerServo.setPosition(SERVO_OFF);
            }

            if(!rightKickerParked) {
                rightKickerServo.setPosition(SERVO_ON);
            } else {
                rightKickerServo.setPosition(SERVO_OFF);
            }
        }

    }

    //Determines if the kickers are in the right position
    public boolean isLeftKickerParked() {

        if(leftKickerDistance<blockedDistance) {
            leftKickerParked=true;
        }
        return leftKickerParked;
    }

    public boolean isRightKickerParked() {

        if(rightKickerDistance<blockedDistance) {
            rightKickerParked=true;
        }
        return rightKickerParked;
    }

    public boolean areKickersParked() {
        if(leftKickerParked && rightKickerParked) {
            areKickersParked=true;
        } else {
            areKickersParked=false;
        }
        return areKickersParked;
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