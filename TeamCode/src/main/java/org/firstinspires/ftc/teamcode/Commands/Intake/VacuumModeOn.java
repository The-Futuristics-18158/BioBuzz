package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that activates vacuum mode: sets vacuum flag, raises the ramp lift,
 * and starts the intake roller running.
 *
 * @author gbuckholtz
 */
public class VacuumModeOn extends CommandBase {

    /**
     * Constructs a VacuumModeOn command, requiring Intake and RampLift subsystems.
     *
     * @author gbuckholtz
     */
    public VacuumModeOn() {
        // Require intake and ramp lift subsystems
        addRequirements(RobotContainer.intake);
        addRequirements(RobotContainer.ramplift);
    }

    /**
     * Sets vacuuming state flag, raises the ramp, and spins up intake.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Enable vacuuming status flag
        RobotContainer.isVacuuming = true;

        // Raise intake ramp
        RobotContainer.ramplift.Raise();

        // Run intake rollers
        RobotContainer.intake.intakeRun();
    }

    /**
     * Periodic execution loop.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
    }

    /**
     * Finishes immediately after initializing state.
     *
     * @author gbuckholtz
     *
     * @return always true
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Called when the command ends.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted
     */
    @Override
    public void end(boolean interrupted) {
    }
}