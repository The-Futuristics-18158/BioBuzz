package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to tilt the shooter hood to the desired angle to control shot trajectory.
 * Valid angle positions range between 0.0 and 0.1.
 *
 * @author superzokabear
 */
public class HoodTilt extends SubsystemBase {

    // Current requested hood servo position
    public static double hoodPosition;
    private Servo TiltServo;

    // Servo movement limits
    public final double maxAngle = 0.1; // Maximum allowable angle position is 0.1
    public final double minAngle = 0.0; // Minimum allowable angle position is 0.0

    /**
     * Initializes the shooter hood tilt subsystem, configures servo direction,
     * and sets default position to minimum angle.
     *
     * @author superzokabear
     */
    public HoodTilt() {
        // Retrieve the tilt servo from the active OpMode's hardware map
        TiltServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "inniOuttiServo");
        TiltServo.setDirection(Servo.Direction.FORWARD);

        // Set default position to minimum angle
        hoodPosition = minAngle;
    }

    /**
     * Method called periodically by the command scheduler to constrain and apply hood tilt servo positions.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        // Ensure hood tilt position remains within safe mechanical limits
        if (hoodPosition > maxAngle)
            hoodPosition = maxAngle;
        if (hoodPosition < minAngle)
            hoodPosition = minAngle;

        // Only update servo position when actively in Autonomous or TeleOp modes
        if (RobotContainer.GetCurrentMode() == RobotContainer.Modes.Auto ||
                RobotContainer.GetCurrentMode() == RobotContainer.Modes.TeleOp) {
            // Apply commanded position to the servo
            TiltServo.setPosition(hoodPosition);
        }
    }

    /**
     * Sets the target position for the shooter hood.
     *
     * @author superzokabear
     *
     * @param position desired servo position between minAngle (0.0) and maxAngle (0.1)
     */
    public void SetHoodPosition(double position) {
        // Update commanded hood position
        hoodPosition = position;
    }

    /**
     * Retrieves the current target position of the shooter hood.
     *
     * @author superzokabear
     *
     * @return current hood servo target position
     */
    public double GetHoodPosition() {
        // Return current commanded position
        return hoodPosition;
    }
}