package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that raises the intake ramp to feed artifacts into the shooter,
 * monitoring the uptake sensor and optional timeout before lowering the ramp upon completion.
 *
 * @author gbuckholtz
 */
public class UptakeRampControle extends CommandBase {

    private ElapsedTime timer;
    private double seconds;
    private boolean timeout;

    /**
     * Constructs an UptakeRampControle command without timeout (finishes only when artifact is detected).
     *
     * @author gbuckholtz
     */
    public UptakeRampControle() {
        this(0.0, false);
    }

    /**
     * Constructs an UptakeRampControle command with a specified timeout in seconds.
     *
     * @author gbuckholtz
     *
     * @param time maximum duration to keep ramp raised in seconds
     */
    public UptakeRampControle(double time) {
        this(time, true);
    }

    /**
     * Constructs an UptakeRampControle command with configurable duration and timeout flag.
     *
     * @author gbuckholtz
     *
     * @param time maximum duration to keep ramp raised in seconds
     * @param timeoutEnabled true to enable timing out after specified duration
     */
    public UptakeRampControle(double time, boolean timeoutEnabled) {
        // Declare subsystem requirements
        addRequirements(RobotContainer.uptakeSensor);
        addRequirements(RobotContainer.kicker);

        timeout = timeoutEnabled;
        timer = new ElapsedTime();
        seconds = time;
    }

    /**
     * Called once when the command begins; raises the intake ramp servo.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Raise ramp to feed artifact into shooter
        RobotContainer.kicker.kick();
    }

    /**
     * Periodic execution logic while ramp remains raised.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Ramp remains raised
    }

    /**
     * Returns true when an artifact is detected by the uptake sensor or the timeout expires.
     *
     * @author gbuckholtz
     *
     * @return true when command is complete; false otherwise
     */
    @Override
    public boolean isFinished() {
        // Finish if artifact detected or timeout exceeded
        return RobotContainer.uptakeSensor.isUpakeArtifactPresent() ||
                (timeout && timer.seconds() > seconds);
    }

    /**
     * Called when the command ends; lowers the ramp back down.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if command was canceled or interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Return ramp to lowered collection position
        RobotContainer.kicker.stop();
    }
}