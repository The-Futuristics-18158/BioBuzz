package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to control the kicker mechanism using continuous rotation servos.
 *
 * @author superzokabear
 */
public class KickerSubsystem extends SubsystemBase {

    // Kicker servo hardware
    private final Servo leftKickerServo;
    private final Servo rightKickerServo;

    private final DistanceSensor leftKickerSensor;
    private final DistanceSensor rightKickerSensor;

    private boolean areKickersParked;

    private final double PARKED_DISTANCE=30.0; //in mm

    private DistanceSensor uptakeSensor_DistanceSensor;

    private double leftDistance;
    private double rightDistance;

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
    public KickerSubsystem() {
        // Retrieve kicker servos from the active OpMode's hardware map
        leftKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftKickerServo");
        rightKickerServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightKickerServo");

        leftKickerSensor= RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "leftKickerSensor");
        rightKickerSensor= RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "rightKickerSensor");

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
        leftDistance=leftKickerSensor.getDistance(DistanceUnit.MM);
        rightDistance=rightKickerSensor.getDistance(DistanceUnit.MM);

        if(kickTimer.milliseconds()>1000.0) {
            park();
        }
        RobotContainer.telemetrySubsystem.addData("Timer in MS", kickTimer.milliseconds());
        RobotContainer.telemetrySubsystem.addData("Left Kicker Distance", leftKickerSensor.getDistance(DistanceUnit.MM));
        RobotContainer.telemetrySubsystem.addData("Right Kicker Distance", rightKickerSensor.getDistance(DistanceUnit.MM));


        if(leftDistance<PARKED_DISTANCE && rightDistance<PARKED_DISTANCE) {
            areKickersParked=true;
        } else {
            areKickersParked=false;
        }
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

    public boolean kickersParked() {
        return areKickersParked;
    }
}