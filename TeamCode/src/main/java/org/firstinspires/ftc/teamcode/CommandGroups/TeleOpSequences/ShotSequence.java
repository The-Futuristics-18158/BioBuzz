package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForKickersParked;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.KickerSubsystem;

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
        addRequirements(RobotContainer.kickerSubsystem);
        addRequirements(RobotContainer.shooter);

        // Prepend delay if specified
        if (shotDelay > 0) {
            addCommands(new Pause(shotDelay));
        }

        addCommands(
                new ShootCommand(), //not tuned, looks to be right

              // new InstantCommand(() -> RobotContainer.intake.intakeStop()),

                //
                new InstantCommand(() -> RobotContainer.kickerSubsystem.kick()),

                // Wait for the kickers to cycle back to parked, with a timeout in case the sensors never report it
                new ParallelRaceGroup(
                        new WaitForKickersParked(),
                        new Pause(3.0)
                )
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
        RobotContainer.kickerSubsystem.stop();
        RobotContainer.intake.intakeStop();
    }
}