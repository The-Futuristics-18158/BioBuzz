package org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command for teleoperated artifact hunting. Uses camera blob tracking to steer the robot
 * toward detected game pieces while running the intake at collection speed.
 *
 * @author gbuckholtz
 */
public class HuntModeCommand extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    private PIDController omegaControl;
    private int finishedCounter;
    private ElapsedTime timer;
    private double seconds;

    /**
     * Constructs a HuntModeCommand with a default 5.0-second timeout.
     *
     * @author gbuckholtz
     */
    public HuntModeCommand() {
        this(5.0);
    }

    /**
     * Constructs a HuntModeCommand with a custom duration in seconds.
     *
     * @author gbuckholtz
     *
     * @param time maximum duration for the hunt command in seconds
     */
    public HuntModeCommand(double time) {
        seconds = time;
    }

    /**
     * Initializes controllers and resets the timeout timer.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
    }

    /**
     * Periodic execution loop for tracking artifacts via camera and steering the robot.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
    }

    /**
     * Determines whether the hunt command has concluded or timed out.
     *
     * @author gbuckholtz
     *
     * @return true when finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Stops the robot and intake when hunt mode finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
    }
}