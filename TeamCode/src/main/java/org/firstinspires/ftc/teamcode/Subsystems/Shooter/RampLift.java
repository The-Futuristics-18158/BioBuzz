package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to raise and lower the intake/shooting ramp.
 * Raised position (1.0) feeds artifacts into the flywheel for shooting;
 * Lowered position (0.44) collects artifacts from the field floor.
 *
 * @author superzokabear
 */
public class RampLift extends SubsystemBase {

    // Ramp lift servo hardware
    private Servo RampServo;

    // Defined servo positions
    private final double RaisedPosition = 1.0;
    private final double LoweredPosition = 0.44;

    /**
     * Initializes the ramp lift subsystem and lowers the ramp to the collection position.
     *
     * @author superzokabear
     */
    public RampLift() {
        // Retrieve ramp lift servo from the active OpMode's hardware map
        RampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rampServo");

        // Set initial ramp position to lowered state
        Lower();
    }

    /**
     * Method called periodically by the command scheduler.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        // Periodic logic if needed
    }

    /**
     * Raises the intake ramp to the upper position to feed artifacts into the flywheel for shooting.
     *
     * @author superzokabear
     */
    public void Raise() {
        // Command servo to fully raised position
        RampServo.setPosition(RaisedPosition);
    }

    /**
     * Lowers the intake ramp to the bottom position to collect artifacts from the ground.
     *
     * @author superzokabear
     */
    public void Lower() {
        // Command servo to lowered collection position
        RampServo.setPosition(LoweredPosition);
    }

    /**
     * Checks whether the intake ramp is currently in the raised position.
     *
     * @author superzokabear
     *
     * @return true if ramp position is greater than or equal to 1.0; false otherwise
     */
    public boolean RampIsUp() {
        // Evaluate if servo position has reached the raised threshold
        return RampServo.getPosition() >= 1.0;
    }
}