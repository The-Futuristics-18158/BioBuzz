package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that deactivates vacuum mode: stops the intake rollers, lowers the ramp lift,
 * and clears the vacuuming state flag.
 *
 * @author gbuckholtz
 */
public class VacuumModeOff extends CommandBase {

    /**
     * Constructs a VacuumModeOff command, requiring Intake and RampLift subsystems.
     *
     * @author gbuckholtz
     */
    public VacuumModeOff() {
        // Require intake and ramp lift subsystems
        addRequirements(RobotContainer.intake);
        addRequirements(RobotContainer.kicker);
    }

    /**
     * Stops intake, lowers ramp, and clears vacuuming flag.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Stop intake roller
        RobotContainer.intake.intakeStop();

        // Lower ramp back down
        RobotContainer.kicker.stop();

        // Clear vacuuming status flag
        RobotContainer.isVacuuming = false;
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
     * Finishes immediately after updating states.
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