package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command to run the intake roller forward while the command is active.
 * Stops the intake when the command ends or is interrupted.
 *
 * @author gbuckholtz
 */
public class IntakeCommand extends CommandBase {

    /**
     * Constructs an IntakeCommand and reserves the Intake subsystem.
     *
     * @author gbuckholtz
     */
    public IntakeCommand() {
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
        // Startup logic if required
    }

    /**
     * Periodically runs the intake forward at normal speed.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Run the intake forward
        RobotContainer.intake.intakeRun();
    }

    /**
     * Runs continuously while scheduled.
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
     * Stops the intake when the command completes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Stop intake roller
        RobotContainer.intake.intakeStop();
    }
}