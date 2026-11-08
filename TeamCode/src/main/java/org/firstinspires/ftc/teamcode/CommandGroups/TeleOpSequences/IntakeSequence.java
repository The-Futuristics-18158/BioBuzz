package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.UptakeRampControle;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Sequential command group for intake collection during TeleOp.
 * Runs drivetrain forward, roller intake, and uptake ramp control in parallel.
 *
 * @author gbuckholtz
 */
public class IntakeSequence extends SequentialCommandGroup {

    /**
     * Constructs an IntakeSequence and declares requirements on drivetrain and intake.
     *
     * @author gbuckholtz
     */
    public IntakeSequence() {
        // Require drivetrain and intake subsystems
        addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.intake);

        addCommands(
                // Drive forward, run intake, and monitor uptake sensor concurrently
                new ParallelCommandGroup(
                        new IntakeCommand(),
                        new UptakeRampControle(),
                        new InstantCommand(() -> RobotContainer.drivesystem.RobotDrive(1.0, 0.0, 0.0))
                )
        );
    }

    /**
     * Lowers the ramp, stops the intake, and halts the drivetrain upon completion or interruption.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // Clean up and stop all actions
        RobotContainer.ramplift.Lower();
        RobotContainer.intake.intakeStop();
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);
    }
}