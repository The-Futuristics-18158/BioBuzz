package org.firstinspires.ftc.teamcode.Commands.Intake.JogBack;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Command to jog the intake backwards briefly to relieve tension or clear artifacts.
 *
 * @author gbuckholtz
 */
public class JogBackIntake extends CommandBase {

    /**
     * Constructs a JogBackIntake command.
     *
     * @author gbuckholtz
     */
    public JogBackIntake() {
    }

    /**
     * Initializes the jog back command.
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
     * Instantaneous jog action finishes immediately.
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