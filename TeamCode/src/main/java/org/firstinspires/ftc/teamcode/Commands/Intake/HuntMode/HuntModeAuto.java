package org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * Autonomous hunt command that maintains an alliance-specific orientation (90 degrees for red, -90 for blue)
 * while running the intake to collect field artifacts within a configurable time window.
 *
 * @author gbuckholtz
 */
public class HuntModeAuto extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    private PIDController omegaControl;
    private PIDController xControl;
    private int finishedCounter;
    private ElapsedTime timer;
    private double seconds;
    private double targetAngle;

    /**
     * Constructs a HuntModeAuto command with a default 5.0-second timeout.
     *
     * @author gbuckholtz
     */
    public HuntModeAuto() {
        this(5.0);
    }

    /**
     * Constructs a HuntModeAuto command with a custom duration in seconds.
     *
     * @author gbuckholtz
     *
     * @param time maximum duration for the hunt command in seconds
     */
    public HuntModeAuto(double time) {
        haveArtifact = false;

        // Declare drivetrain and intake subsystem requirements
        addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.intake);

        omegaControl = new PIDController(0.05, 0.0005, 0.01);
        xControl = new PIDController(0.0075, 0.0, 0.0);
        timer = new ElapsedTime();
        seconds = time;
    }

    /**
     * Resets controllers, timer, and configures target heading based on alliance color.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        omegaControl.reset();
        xControl.reset();
        finishedCounter = 0;
        timer.reset();

        // 90 degrees for red alliance, -90 degrees for blue alliance
        if (RobotContainer.isRedAlliance()) {
            targetAngle = 90.0;
        } else {
            targetAngle = -90.0;
        }
    }

    /**
     * Periodically runs intake, calculates heading correction via PID, and checks timeout.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        double x_speed;
        double y_speed;
        double omega_speed;

        // Check if timeout reached
        if (timer.seconds() >= seconds) {
            RobotContainer.intake.intakeStop();
            finishedCounter++;
        } else {
            finishedCounter = 0;
            RobotContainer.intake.intakeRun();
        }

        // Heading correction PID
        double angleError = Utils.AngleDifference(RobotContainer.gyro.getYawAngle(), targetAngle);
        omega_speed = -omegaControl.calculate(angleError);
    }

    /**
     * Checks if the hunt mode command has concluded.
     *
     * @author gbuckholtz
     *
     * @return true when finished counter reaches threshold
     */
    @Override
    public boolean isFinished() {
        return (finishedCounter >= 2);
    }

    /**
     * Halts drivetrain and stops intake when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0, 0, 0);
        RobotContainer.intake.intakeStop();
    }
}