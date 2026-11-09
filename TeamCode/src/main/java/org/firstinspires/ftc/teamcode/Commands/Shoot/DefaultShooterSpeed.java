package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command to maintain default shooter idle speed when not actively targeting or shooting.
 *
 * @author gbuckholtz
 */
public class DefaultShooterSpeed extends CommandBase {

    /**
     * Constructs a DefaultShooterSpeed command.
     *
     * @author gbuckholtz
     */
    public DefaultShooterSpeed() {
        // Add subsystem requirements if needed
    }

    /**
     * Called once when command begins.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Initialization if needed
    }

    /**
     * Periodically updates the default shooter speed.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Set idle flywheel speed when active
    }

    /**
     * Default command runs continuously until interrupted.
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
     * Called when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by another command
     */
    @Override
    public void end(boolean interrupted) {
        // Cleanup if required
    }
}