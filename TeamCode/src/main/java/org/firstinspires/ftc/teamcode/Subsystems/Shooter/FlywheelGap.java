package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BallColorSensor;

/**
 * Subsystem controlling the top servo on the shooter ("inniOuttiServo")
 * to adjust the flywheel gap (compression spacing) based on ball size.
 * <p>
 * <b>Teaching Concept - Mechanical Ball Sizing & Compression:</b>
 * Rather than tilting a hood to change trajectory, this mechanism resizes the physical
 * gap between the shooter wheels to match the game element:
 * <ul>
 *   <li><b>Pollen:</b> Smaller ball size requires a narrower gap (higher servo extension)
 *       to grip the ball properly and achieve consistent exit velocity.</li>
 *   <li><b>Nectar:</b> Larger ball size requires a wider gap (lower servo extension)
 *       to prevent jamming or stalling the flywheel motors.</li>
 * </ul>
 *
 * @author superzokabear, gbuckholtz
 */
public class FlywheelGap extends SubsystemBase {

    // Current requested servo position for the flywheel gap
    public static double targetGap;
    private Servo gapServo;

    // Servo movement physical limits
    public static final double MAX_GAP = 0.1; // Maximum allowable position
    public static final double MIN_GAP = 0.0; // Minimum allowable position

    // Preset positions for flywheel gap sizing based on ball type:
    // Pollen balls are smaller, requiring a narrower gap (higher servo extension / more compression)
    public static final double GAP_POLLEN = 0.08;
    // Nectar balls are larger, requiring a wider gap (lower servo extension / less compression)
    public static final double GAP_NECTAR = 0.02;
    // Fail-Safe: Always default to Nectar (wider gap) to protect components if ball type cannot be determined
    public static final double GAP_DEFAULT = GAP_NECTAR;

    // Tolerance for verifying servo has reached target position
    private static final double POSITION_TOLERANCE = 0.005;

    /**
     * Initializes the flywheel gap subsystem, configures the servo direction,
     * and sets default position to standard flywheel gap.
     *
     * @author superzokabear, gbuckholtz
     */
    public FlywheelGap() {
        try {
            // Retrieve the gap servo from the active OpMode's hardware map
            gapServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "inniOuttiServo");
            gapServo.setDirection(Servo.Direction.FORWARD);
        } catch (Exception e) {
            if (RobotContainer.telemetrySubsystem != null) {
                RobotContainer.telemetrySubsystem.addData("FlywheelGap Servo Init Error", e.getMessage());
            }
        }

        // Set default position to standard flywheel gap
        targetGap = GAP_DEFAULT;
    }

    /**
     * Method called periodically by the command scheduler to constrain and apply gap servo positions.
     *
     * @author superzokabear, gbuckholtz
     */
    @Override
    public void periodic() {
        // Ensure gap position remains within safe mechanical limits
        if (targetGap > MAX_GAP)
            targetGap = MAX_GAP;
        if (targetGap < MIN_GAP)
            targetGap = MIN_GAP;

        // Only update servo position when actively in Autonomous or TeleOp modes
        if (RobotContainer.GetCurrentMode() == RobotContainer.Modes.Auto ||
                RobotContainer.GetCurrentMode() == RobotContainer.Modes.TeleOp) {
            if (gapServo != null) {
                gapServo.setPosition(targetGap);
            }
        }
    }

    /**
     * Sets the top servo position to adjust the flywheel gap according to the detected ball type.
     * Selects a narrower gap for Pollen balls and defaults to the wider gap for Nectar or unknown elements.
     * <p>
     * <b>Fail-Safe Design:</b> Defaults to {@link #GAP_NECTAR} (wider gap) if ball type is unknown
     * or undetected, preventing potential mechanical jamming or component damage if a larger ball (Nectar) is fed.
     *
     * @author gbuckholtz
     *
     * @param ballType the classified {@link BallColorSensor.BallType}
     */
    public void setGapForBall(BallColorSensor.BallType ballType) {
        if (ballType == BallColorSensor.BallType.POLLEN) {
            setGap(GAP_POLLEN);
        } else {
            // NECTAR, UNKNOWN, or NONE: safely default to the wider gap
            setGap(GAP_NECTAR);
        }
    }

    /**
     * Sets the target flywheel gap position.
     *
     * @author gbuckholtz
     *
     * @param position desired servo position between MIN_GAP and MAX_GAP
     */
    public void setGap(double position) {
        targetGap = position;
    }

    /**
     * Retrieves the current target position of the flywheel gap.
     *
     * @author gbuckholtz
     *
     * @return current target gap servo position
     */
    public double getGap() {
        return targetGap;
    }

    /**
     * Checks if the servo has reached its commanded target gap position within tolerance.
     *
     * @author gbuckholtz
     *
     * @return true if current servo position is close to target; false otherwise
     */
    public boolean isAtTargetGap() {
        if (gapServo == null) return true;
        return Math.abs(gapServo.getPosition() - targetGap) <= POSITION_TOLERANCE;
    }
}
