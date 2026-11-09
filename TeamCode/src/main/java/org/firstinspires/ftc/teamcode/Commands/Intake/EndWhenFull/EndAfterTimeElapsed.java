package org.firstinspires.ftc.teamcode.Commands.Intake.EndWhenFull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command that finishes execution after a predefined duration has elapsed
 * to provide a fixed-time intake window.
 *
 * @author knutt5
 */
public class EndAfterTimeElapsed extends CommandBase {
    private ElapsedTime timer;
    private boolean finished;

    /**
     * Constructs an EndAfterTimeElapsed command and initializes the elapsed timer.
     *
     * @author knutt5
     */
    public EndAfterTimeElapsed() {
        timer = new ElapsedTime();
        timer.reset();
    }

    /**
     * Resets the timer and finished state when the command is scheduled.
     *
     * @author knutt5
     */
    @Override
    public void initialize() {
        timer.reset();
        finished = false;
    }

    /**
     * Periodically monitors whether the elapsed intake time exceeds the timeout threshold (2.5 seconds).
     *
     * @author knutt5
     */
    @Override
    public void execute() {
        if (timer.seconds() > 2.5) {
            finished = true;
        }
    }

    /**
     * Checks if the time window has finished.
     *
     * @author knutt5
     *
     * @return true if elapsed time exceeds 2.5s, false otherwise
     */
    @Override
    public boolean isFinished() {
        return finished;
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
    }
}