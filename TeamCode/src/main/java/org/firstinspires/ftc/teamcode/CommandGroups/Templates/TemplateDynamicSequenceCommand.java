package org.firstinspires.ftc.teamcode.CommandGroups.Templates;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

/**
 * Template class demonstrating how to dynamically construct and schedule
 * a SequentialCommandGroup at runtime during the initialize phase.
 *
 * @author gbuckholtz
 */
public class TemplateDynamicSequenceCommand extends CommandBase {

    // The sequential command group created and executed dynamically
    private SequentialCommandGroup cmd;

    /**
     * Constructs a TemplateDynamicSequenceCommand.
     *
     * @author gbuckholtz
     */
    public TemplateDynamicSequenceCommand() {
    }

    /**
     * Dynamically builds the command sequence based on runtime state and initializes it.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Instantiate the dynamic sequence
        cmd = new SequentialCommandGroup();

        // Initialize and start the sequence
        cmd.initialize();
    }

    /**
     * Periodically executes the dynamic command sequence.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        cmd.execute();
    }

    /**
     * Checks if the dynamically constructed command sequence has completed.
     *
     * @author gbuckholtz
     *
     * @return true when the sequence is finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    /**
     * Ends the dynamic sequence when finished or interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }
}