package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command to reverse the intake rollers to clear artifact jams or eject artifacts.
 * Stops the intake when the command ends or is interrupted.
 *
 * @author gbuckholtz
 */
public class ReverseIntakeCommand extends CommandBase {

    /**
     * Constructs a ReverseIntakeCommand and reserves the Intake subsystem.
     *
     * @author gbuckholtz
     */
    public ReverseIntakeCommand() {
        // Declare intake subsystem requirement
        addRequirements(RobotContainer.intake);
    }

    /**
     * Called once when the command begins.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Initialization if needed
    }

    /**
     * Periodically commands the intake to run in reverse.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Run intake in reverse
        RobotContainer.intake.intakeReverse();
    }

    /**
     * Runs continuously while scheduled until interrupted.
     *
     * @author gbuckholtz
     *
     * @return always false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Stops the intake when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the intake motor
        RobotContainer.intake.intakeStop();
    }
}