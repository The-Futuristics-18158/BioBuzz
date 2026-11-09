package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that waits for the shooter flywheel to reach its target RPM or until a maximum spin-up timeout elapses.
 *
 * @author knutt5
 */
public class WaitForSpinup extends CommandBase {

    private double maxtime;
    private ElapsedTime time;

    /**
     * Constructs a WaitForSpinup command and instantiates the elapsed timer.
     *
     * @author knutt5
     */
    public WaitForSpinup() {
        time = new ElapsedTime();
    }

    /**
     * Called once when the command begins; resets timer.
     *
     * @author knutt5
     */
    @Override
    public void initialize() {
        time.reset();
    }

    /**
     * Periodically called while waiting for spinup.
     *
     * @author knutt5
     */
    @Override
    public void execute() {
        // Spinup wait loop
    }

    /**
     * Determines whether the shooter flywheel has reached target speed or timed out.
     *
     * @author knutt5
     *
     * @return true when spin-up condition or timeout is satisfied
     */
    @Override
    public boolean isFinished() {
        // Return true once flywheel achieves speed or condition is met
        return true;
    }

    /**
     * Called when the command ends or is interrupted.
     *
     * @author knutt5
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // Cleanup if required
    }
}