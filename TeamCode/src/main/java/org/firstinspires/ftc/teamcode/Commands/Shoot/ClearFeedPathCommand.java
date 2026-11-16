package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that rotates the kicker paddles until both side distance sensors indicate
 * that the fins are clear of the feed corridor, leaving an unobstructed path for incoming balls.
 * <p>
 * <b>Teaching Concept - Sensor-Feedback Positioning:</b>
 * Rather than guessing timing with an open-loop delay (which varies as battery voltage changes),
 * this command monitors the side distance sensors to stop the continuous rotation paddles
 * at the exact moment the feed path is clear. A fail-safe timeout prevents the robot from
 * hanging if a sensor is disconnected.
 *
 * @author gbuckholtz
 */
public class ClearFeedPathCommand extends CommandBase {

    private final ElapsedTime timer;
    private final double timeoutSeconds;

    /**
     * Constructs a ClearFeedPathCommand with a default timeout of 1.0 second.
     *
     * @author gbuckholtz
     */
    public ClearFeedPathCommand() {
        this(1.0);
    }

    /**
     * Constructs a ClearFeedPathCommand with a specified maximum timeout.
     *
     * @author gbuckholtz
     *
     * @param timeoutSeconds maximum duration to rotate while attempting to clear
     */
    public ClearFeedPathCommand(double timeoutSeconds) {
        addRequirements(RobotContainer.kicker);
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
    }

    /**
     * Called once when the command begins; starts rotating the paddles to clear the path.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        timer.reset();
        RobotContainer.kicker.clearFeedPath();
    }

    /**
     * Periodically updates rotation towards the clear position.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        RobotContainer.kicker.clearFeedPath();
    }

    /**
     * Finishes when both side sensors detect clear passage or the fail-safe timeout expires.
     *
     * @author gbuckholtz
     *
     * @return true when feed path is clear or timeout elapsed
     */
    @Override
    public boolean isFinished() {
        return RobotContainer.kicker.isFeedPathClear() || timer.seconds() >= timeoutSeconds;
    }

    /**
     * Called when the command finishes or is interrupted; ensures kicker servos are stopped.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by another command
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.kicker.stop();
    }
}
