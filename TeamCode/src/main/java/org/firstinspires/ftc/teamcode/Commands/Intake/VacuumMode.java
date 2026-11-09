package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that toggles vacuum mode, synchronizing intake roller running and ramp lifting.
 *
 * @author gbuckholtz
 */
public class VacuumMode extends CommandBase {

    private static boolean onoff;

    /**
     * Constructs a VacuumMode toggle command, requiring both Intake and RampLift subsystems.
     *
     * @author gbuckholtz
     */
    public VacuumMode() {
        // Require intake and ramp lift subsystems
        addRequirements(RobotContainer.intake);
        addRequirements(RobotContainer.ramplift);

        onoff = false;
    }

    /**
     * Toggles intake and ramp lift states based on current vacuum state.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        if (!onoff) {
            // Start vacuuming: run intake and raise ramp
            RobotContainer.intake.intakeRun();
            RobotContainer.ramplift.Raise();
        } else {
            // Stop vacuuming: stop intake and lower ramp
            RobotContainer.intake.intakeStop();
            RobotContainer.ramplift.Lower();
        }
        // Invert toggle state
        onoff = !onoff;
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
     * Instantaneous toggle finishes immediately after initialize.
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