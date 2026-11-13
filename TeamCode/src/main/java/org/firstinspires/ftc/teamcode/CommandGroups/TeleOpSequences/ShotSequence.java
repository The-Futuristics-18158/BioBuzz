package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Sequential command group for executing a shot into the alliance goal.
 * Lifts the ramp to index the artifact, delays for positioning, feeds with the intake roller,
 * and resets the ramp and intake upon completion.
 *
 * @author gbuckholtz
 */
public class ShotSequence extends SequentialCommandGroup {

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
        // Require intake and ramp lift subsystems
        addRequirements(RobotContainer.intake);
        addRequirements(RobotContainer.kicker);

        // Prepend delay if specified
        if (shotDelay > 0) {
            addCommands(new Pause(shotDelay));
        }

        addCommands(
                // Raise ramp to position artifact
                new InstantCommand(() -> RobotContainer.kicker.kick()),

                // Wait for ramp to lift
                new Pause(0.25),

                // Wait for shot completion
                new Pause(1.90)
        );
    }

    /**
     * Lowers the ramp and stops the intake motor upon completion or interruption.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.kicker.stop();
        RobotContainer.intake.intakeStop();
    }
}