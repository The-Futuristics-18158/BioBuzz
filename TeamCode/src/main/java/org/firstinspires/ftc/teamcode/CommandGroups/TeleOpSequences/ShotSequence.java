package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Shoot.ClearFeedPathCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.FlywheelGap;

/**
 * Sequential command group for executing a complete, sensor-driven shot sequence into the goal.
 * <p>
 * <b>Teaching Concept - Declarative Command Composition:</b>
 * This command group demonstrates how distinct robot actions can be chained sequentially:
 * <ol>
 *   <li><b>Targeting & Setup:</b> Checks the top ball sensor, adjusts the top servo to set the
 *       correct flywheel gap (narrower for Pollen, wider/default for Nectar), and starts flywheel spin-up.</li>
 *   <li><b>Wait for Flywheel Spin-Up:</b> Waits using {@link WaitForSpinup} until the flywheel
 *       velocity is within tolerance or a fail-safe timeout expires.</li>
 *   <li><b>Servo Settle:</b> Allows a brief pause for the top sizing servo to reach mechanical position.</li>
 *   <li><b>Kicker Advance:</b> Advances the kicker paddles to kick the ball into the flywheels.</li>
 *   <li><b>Clear Feed Corridor:</b> Executes {@link ClearFeedPathCommand} which uses the two side
 *       distance sensors to stop the paddles only after the fins rotate clear of the feed path.</li>
 * </ol>
 *
 * @author gbuckholtz
 */
public class ShotSequence extends SequentialCommandGroup {

    private static final double DEFAULT_SHOOTING_RPM = 4500.0;
    private static final double SPINUP_TOLERANCE_RPM = 100.0;
    private static final double SPINUP_TIMEOUT_SECONDS = 1.5;
    private static final double SERVO_SETTLE_SECONDS = 0.15;
    private static final double KICK_PROPEL_SECONDS = 0.40;

    /**
     * Constructs a standard ShotSequence with default timing.
     *
     * @author gbuckholtz
     */
    public ShotSequence() {
        this(0.0);
    }

    /**
     * Constructs a ShotSequence with an optional initial delay before firing.
     *
     * @author gbuckholtz
     *
     * @param shotDelay delay in seconds before commencing the shot sequence
     */
    public ShotSequence(double shotDelay) {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.flywheelGap);
        addRequirements(RobotContainer.kicker);
        addRequirements(RobotContainer.intake);

        // Optional pre-shot delay (e.g., waiting for robot alignment or drivetrain settling)
        if (shotDelay > 0) {
            addCommands(new Pause(shotDelay));
        }

        addCommands(
                // Step 1: Detect ball and initiate flywheel spinup and top servo gap sizing
                new InstantCommand(() -> {
                    // Check top color sensor for ball presence and type.
                    // Fail-Safe: Default to NECTAR (the larger ball / wider gap) to prevent
                    // potential mechanical jamming/damage if sensor is unreachable or uncertain.
                    BallColorSensor.BallType detectedType = RobotContainer.ballSensor.getBallType();

                    BallColorSensor.BallType ballType = (detectedType == BallColorSensor.BallType.POLLEN)
                            ? BallColorSensor.BallType.POLLEN
                            : BallColorSensor.BallType.NECTAR;

                    // Set top servo flywheel gap: narrower gap for Pollen, wider gap for Nectar
                    RobotContainer.flywheelGap.setGapForBall(ballType);

                    // Spin up flywheels to shooting RPM
                    RobotContainer.shooter.SetFlywheelSpeed(DEFAULT_SHOOTING_RPM);
                }),

                // Step 2: Wait for flywheel to reach target speed (with fail-safe timeout)
                new WaitForSpinup(SPINUP_TIMEOUT_SECONDS, SPINUP_TOLERANCE_RPM),

                // Step 3: Brief delay to ensure top servo has mechanically settled
                new Pause(SERVO_SETTLE_SECONDS),

                // Step 4: Advance kickers to propel the ball into the flywheels
                new InstantCommand(() -> RobotContainer.kicker.advanceToShoot()),

                // Step 5: Allow paddles sufficient time to complete the kick stroke
                new Pause(KICK_PROPEL_SECONDS),

                // Step 6: Rotate kicker until side distance sensors detect fins are clear of the feed path
                new ClearFeedPathCommand(1.0)
        );
    }

    /**
     * Resets subsystems upon completion or interruption.
     * Stops kicker servos, returns flywheels to coast, and restores default wheel gap.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.kicker.stop();
        RobotContainer.shooter.SetFlywheelSpeed(0.0);
        RobotContainer.flywheelGap.setGap(FlywheelGap.GAP_DEFAULT);
        RobotContainer.intake.intakeStop();
    }
}
