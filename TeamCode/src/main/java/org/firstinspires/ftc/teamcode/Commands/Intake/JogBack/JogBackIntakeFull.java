package org.firstinspires.ftc.teamcode.Commands.Intake.JogBack;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Command to jog the intake backward by a full calibrated number of shaft rotations
 * using proportional position control to clear jam conditions.
 *
 * @author gbuckholtz
 */
public class JogBackIntakeFull extends CommandBase {

    private double TargetPosition;

    // Unit for P-gain: motor rps / motor ticks
    // 100 is proportional gain; (1/28) * (1/5.0) accounts for motor ticks and 5:1 gearbox
    private final double Pgain = 100.0 * (1.0 / 28.0) * (1.0 / 5.0);

    // Number of intake shaft turns to jog back (0.15 turns)
    private final double JogBackTurns = 0.15;

    /**
     * Constructs a JogBackIntakeFull command.
     *
     * @author gbuckholtz
     */
    public JogBackIntakeFull() {
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
     * Periodically computes position error and commands intake reverse speed.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
    }

    /**
     * Evaluates whether the jog back operation has completed.
     *
     * @author gbuckholtz
     *
     * @return true when finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return false;
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