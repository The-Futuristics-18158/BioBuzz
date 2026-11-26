package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.SimplifiedFlywheel;

/**
 * Template class for creating new FTCLib commands.
 * Outlines the standard command lifecycle: initialize, execute, isFinished, and end.
 *
 * @author gbuckholtz
 */
public class ShootCommand extends CommandBase {

    SimplifiedFlywheel shooter;

    private double myFlywheelSpeed;

    boolean atSpeed;

    /**
     * Constructs a new TemplateCommand.
     * Declare subsystem requirements here using addRequirements().
     *
     * @author gbuckholtz
     */
    public ShootCommand() {
        // Add required subsystems (e.g., addRequirements(RobotContainer.drivesystem);)
        addRequirements(RobotContainer.shooter);
        shooter = RobotContainer.shooter;
    }

    /**
     * Called once when the command is initially scheduled and started.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Reset timers or initial states
        atSpeed=false;
        setSpeed(1500.0);
    }

    /**
     * Called periodically while the command is actively scheduled.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Periodic control logic
        double currentFlywheelSpeed= shooter.GetFlywheelSpeed();
        RobotContainer.telemetrySubsystem.addData("Current Flywheel Speed", currentFlywheelSpeed);

        if(currentFlywheelSpeed>(myFlywheelSpeed-50)|| currentFlywheelSpeed<(myFlywheelSpeed+50)) {
            atSpeed=true;
        }
    }

    public void setSpeed(double targetSpeed) {
        myFlywheelSpeed=targetSpeed;
        shooter.SetFlywheelSpeed(targetSpeed);
    }

    /**
     * Evaluates whether the command has completed its execution.
     *
     * @author gbuckholtz
     *
     * @return true when finished, false to continue executing
     */
    @Override
    public boolean isFinished() {
        return atSpeed;
    }

    /**
     * Called once when the command finishes or is interrupted by another command.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if the command was interrupted by the scheduler, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        // Stop motors or reset subsystems
    }
}