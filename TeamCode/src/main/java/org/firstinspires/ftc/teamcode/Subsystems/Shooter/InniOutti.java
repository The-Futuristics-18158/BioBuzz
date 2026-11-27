package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to tilt the shooter hood to the desired angle to control shot trajectory.
 * Valid angle positions range between 0.0 and 0.1.
 *
 * @author superzokabear
 */
public class InniOutti extends SubsystemBase {

    private final double POLLEN_GAP=0.34;
    private final double NECTAR_GAP=0.75;
    private final double SAFE_POSITION=0.4;
    private double target_gap;

    public static final int YELLOW=1;
    public static final int RED=2;
    public static final int BLUE=3;

    // Current requested hood servo position
    public static double hoodPosition;
    private Servo inniOuttiServo;
    private ColorSensor uptakeSensor;

    // Servo movement limits
    public final double maxAngle = 0.1; // Maximum allowable angle position is 0.1
    public final double minAngle = 0.0; // Minimum allowable angle position is 0.0

    public final double pollenAngle=0.75;
    public final double nectarAngle=0;

    private int finalColour=BLUE;

    /**
     * Initializes the shooter hood tilt subsystem, configures servo direction,
     * and sets default position to minimum angle.
     *
     * @author superzokabear
     */
    public InniOutti() {
        // Initializer
        inniOuttiServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "inniOuttiServo");
        inniOuttiServo.setDirection(Servo.Direction.FORWARD);
        target_gap=SAFE_POSITION;

        uptakeSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "uptakeSensor");
        ((NormalizedColorSensor) uptakeSensor).setGain(1);

    }

//    public setBallGap() {
//
//    }

    private int getColour() {
        int ballRedness;
        int ballBlueness;
        int ballGreenness;


        RobotContainer.telemetrySubsystem.addData("Ball Uptake Alphaness", uptakeSensor.alpha());
        RobotContainer.telemetrySubsystem.addData("Ball Uptake Redness", uptakeSensor.red());
        RobotContainer.telemetrySubsystem.addData("Ball Uptake Greenness", uptakeSensor.green());
        RobotContainer.telemetrySubsystem.addData("Ball Uptake Blueness", uptakeSensor.blue());


        ballRedness=uptakeSensor.red();
        ballBlueness=uptakeSensor.blue();
        ballGreenness=uptakeSensor.green();

        //Nothing: alpha=49, red=29, green=55, blue=65
        //Blue: alpha=159, red=59, green=121, blue=299
        //Yellow: alpha=129, red=112, green=180, blue=97
        //Red(ESTIMATION): alpha=159, red=140, green=150, blue=120

        if(ballPresent()) {

            if(ballBlueness>220) {
                finalColour=BLUE;
            } else {
                if(ballGreenness>90) {
                    finalColour=YELLOW;
                } else {
                    finalColour=RED;
                }
            }
        }
        return finalColour;
    }

    private boolean ballPresent() {

        boolean ballHere=false;
        int ballAlphaness;
        ballAlphaness= uptakeSensor.alpha();

        if(ballAlphaness<60) {
            ballHere=false;
        } else {
            ballHere=true;
        }
        return ballHere;
    }

    public int getCurrentColour() {
        return finalColour;
    }

    /**
     * Method called periodically by the command scheduler to constrain and apply hood tilt servo positions.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
            getColour();
            if(finalColour==BLUE || finalColour==RED) {
                inniOuttiServo.setPosition(nectarAngle);
            }

            if(finalColour==YELLOW) {
                inniOuttiServo.setPosition(pollenAngle);
            }
        }
    }
