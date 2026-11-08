package org.firstinspires.ftc.teamcode.Commands.Utility;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command that pauses command group execution for a specified duration in seconds.
 *
 * @author gbuckholtz
 */
public class Pause extends CommandBase {
    private ElapsedTime delayTimer;
    private double delayTime;

    /**
     * Constructs a Pause command with the specified delay duration.
     *
     * @author gbuckholtz
     *
     * @param delay time duration to pause in seconds
     */
    public Pause(double delay) {
        delayTime = delay;
        delayTimer = new ElapsedTime();
    }

    /**
     * Resets the delay timer when the command is scheduled.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Reset timer
        delayTimer.reset();
    }

    /**
     * Periodic execution loop while waiting.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Idling until elapsed time reaches delay duration
    }

    /**
     * Determines whether the specified delay time has elapsed.
     *
     * @author gbuckholtz
     *
     * @return true if elapsed time >= delay time, false otherwise
     */
    @Override
    public boolean isFinished() {
        // Check if duration has expired
        return delayTimer.seconds() >= delayTime;
    }

    /**
     * Called once when the pause delay finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if command was canceled or interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }
}