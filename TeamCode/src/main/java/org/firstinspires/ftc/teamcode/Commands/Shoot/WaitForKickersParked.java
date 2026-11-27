package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Finishes once the kickers have left the parked position and returned to it.
 * The kickers start parked, so the command waits for them to move before accepting parked as done.
 *
 * @author gbuckholtz
 */
public class WaitForKickersParked extends CommandBase {

    private boolean sawNotParked;

    @Override
    public void initialize() {
        sawNotParked = false;
    }

    @Override
    public boolean isFinished() {
        if (!RobotContainer.kickerSubsystem.kickersParked()) {
            sawNotParked = true;
        }
        return sawNotParked && RobotContainer.kickerSubsystem.kickersParked();
    }
}
