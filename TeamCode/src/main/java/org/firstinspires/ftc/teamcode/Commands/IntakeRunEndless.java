package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that runs the intake roller continuously until canceled or interrupted.
 * Typically combined in parallel race groups or interrupted by subsequent commands.
 *
 * @author gbuckholtz
 */
public class IntakeRunEndless extends CommandBase {

    /**
     * Constructs the IntakeRunEndless command and reserves the Intake subsystem.
     *
     * @author gbuckholtz
     */
    public IntakeRunEndless() {
        // Require the intake subsystem
        addRequirements(RobotContainer.intake);
    }

    /**
     * Starts running the intake forward at standard collection speed.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Start intake motor
        RobotContainer.intake.intakeRun();
    }

    /**
     * Periodic execution loop while intake runs continuously.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Intake runs via subsystem controller
    }

    /**
     * This command runs continuously and does not finish on its own.
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
     * Stops the intake motor when interrupted or canceled.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if command was interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // Shut off intake motor
        RobotContainer.intake.intakeStop();
    }
}