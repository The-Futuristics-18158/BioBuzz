package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to Tilt Hood to Desired Angle
 * 0.0 <= Angle <= 0.1
 *
 * @author superzokabear
 */
//@Configurable
public class HoodTilt extends SubsystemBase {

    // Local objects and variables here
    public static double hoodPosition; // use public static when using panels, otherwise make private
    private Servo TiltServo;

    // Servo movement limits
    public final double maxAngle = 0.1;// max angle is 0.1
    public final double minAngle = 0.0; // min angle is 0.0
    // note: use 0.3 to re-calibrate hood gear position
    // 0.3 is at position where hood gear just detaches from servo gear.

    /** Place code here to initialize subsystem */
    public HoodTilt() {
        // create servo and configure
        TiltServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "inniOuttiServo");
        TiltServo.setDirection(Servo.Direction.FORWARD);

        // set default position to min angle
        hoodPosition = minAngle;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // ensure hood tilt within allowable limits
        // this code needed for panels so user does not enter value outside range
        if (hoodPosition>maxAngle)
            hoodPosition = maxAngle;
        if (hoodPosition<minAngle)
            hoodPosition = minAngle;

        // only change hood position if robot in auto or teleop modes
        if (RobotContainer.GetCurrentMode() == RobotContainer.Modes.Auto ||
                RobotContainer.GetCurrentMode() == RobotContainer.Modes.TeleOp)

            // set hood position
            TiltServo.setPosition(hoodPosition);
    }


    /**Sets shooter hood position
     * @param position Desired position between 0.3 and 1.0
     *      Last updated for Canada Cup Robot
     */
    public void SetHoodPosition(double position){
        hoodPosition = position;
    }

    /**Returns shooter hood position
     */
    public double GetHoodPosition() {
        return hoodPosition;
    }

}