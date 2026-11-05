package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class VacuumModeOff extends CommandBase {

    // constructor
    public VacuumModeOff() {

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.intake);
        addRequirements(RobotContainer.ramplift);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.intake.intakeStop();
        RobotContainer.ramplift.Lower();
        RobotContainer.isVacuuming = false;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}