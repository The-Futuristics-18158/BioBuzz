package org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous artifact hunt command that maintains a fixed 0-degree robot heading
 * while actively collecting field artifacts.
 *
 * @author gbuckholtz
 */
public class HuntModeAutoZeroAngle extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    private PIDController omegaControl;
    private PIDController xControl;
    private int finishedCounter;
    private ElapsedTime timer;
    private double seconds;
    private double targetAngle;

    /**
     * Constructs a HuntModeAutoZeroAngle command with a default 5.0-second timeout.
     *
     * @author gbuckholtz
     */
    public HuntModeAutoZeroAngle() {
        this(5.0);
    }

    /**
     * Constructs a HuntModeAutoZeroAngle command with a custom duration in seconds.
     *
     * @author gbuckholtz
     *
     * @param time maximum duration for the hunt command in seconds
     */
    public HuntModeAutoZeroAngle(double time) {
        seconds = time;
    }

    /**
     * Resets controllers and timers upon initialization.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
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
     * Determines whether the hunt command has concluded.
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
     * Called when the command ends or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
    }
}