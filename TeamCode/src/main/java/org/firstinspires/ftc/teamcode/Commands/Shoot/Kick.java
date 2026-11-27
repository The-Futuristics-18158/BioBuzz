package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.KickerSubsystem;

/**
 * Template class for creating new FTCLib commands.
 * Outlines the standard command lifecycle: initialize, execute, isFinished, and end.
 *
 * @author gbuckholtz
 */
public class Kick extends CommandBase {
    KickerSubsystem kickerSubsystem;

    /**
     * Constructs a new TemplateCommand.
     * Declare subsystem requirements here using addRequirements().
     *
     * @author gbuckholtz
     */
    public Kick() {
        // Add the kicker RobotContainer
        addRequirements(RobotContainer.kickerSubsystem);
        kickerSubsystem =RobotContainer.kickerSubsystem;
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
        kickerSubsystem.stop();
    }
}