package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Template class for creating new FTCLib commands.
 * Outlines the standard command lifecycle: initialize, execute, isFinished, and end.
 *
 * @author gbuckholtz
 */
public class TemplateCommand extends CommandBase {

    /**
     * Constructs a new TemplateCommand.
     * Declare subsystem requirements here using addRequirements().
     *
     * @author gbuckholtz
     */
    public TemplateCommand() {
        // Add required subsystems (e.g., addRequirements(RobotContainer.drivesystem);)
    }

    /**
     * Called once when the command is initially scheduled and started.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Reset timers or initial states
    }

    /**
     * Called periodically while the command is actively scheduled.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Periodic control logic
    }

    /**
     * Evaluates whether the command has completed its execution.
     *
     * @author gbuckholtz
     *
     * @return true when finished, false to continue executing
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Called once when the command finishes or is interrupted by another command.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if the command was interrupted by the scheduler, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        // Stop motors or reset subsystems
    }
}