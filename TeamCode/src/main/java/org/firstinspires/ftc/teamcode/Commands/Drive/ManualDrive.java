package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/**
 * Command for teleoperated field-oriented mecanum driving.
 * Supports joystick dead-zoning, right-trigger boost speed scaling, alliance orientation inversion,
 * and automatic gyro heading-hold PID control when no rotational input is commanded.
 *
 * @author gbuckholtz
 */
public class ManualDrive extends CommandBase {

    // PID controller used to counteract rotational drift due to wheel misalignment
    private PIDController m_headingPID = new PIDController(0.06, 0.001, 0);
    private Double m_PIDTarget = null;
    private long m_pidDelay = -1;

    // Power scaling factors
    private double powerFactor;
    private double basePowerFacter = 0.65;
    private double boostPowerFacter = 0.35;
    private double slowPowerFactor = 0.4;

    /**
     * Constructs a ManualDrive command requiring the drivetrain subsystem.
     *
     * @author gbuckholtz
     */
    public ManualDrive() {
        // Require drivetrain
        addRequirements(RobotContainer.drivesystem);
    }

    /**
     * Resets the heading target and PID delay upon command startup.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        m_PIDTarget = null;
        m_pidDelay = 10;
    }

    /**
     * Periodically processes gamepad inputs, applies dead-zones, scales power,
     * corrects for heading drift, and drives the robot in field coordinates.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        // Read joystick inputs from gamepad 1
        double dX = RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        double dY = -RobotContainer.ActiveOpMode.gamepad1.left_stick_y;
        double omega = -3.0 * RobotContainer.ActiveOpMode.gamepad1.right_stick_x;
        double speedTrigger = RobotContainer.ActiveOpMode.gamepad1.right_trigger;

        // Invert X and Y for blue alliance so forward always matches driver perspective
        if (!RobotContainer.isRedAlliance) {
            dX = dX * -1;
            dY = dY * -1;
        }

        // Apply dead-zoning to prevent unwanted creep
        dX = Math.abs(dX) > 0.05 ? dX : 0;
        dY = Math.abs(dY) > 0.05 ? dY : 0;
        omega = Math.abs(omega) > 0.05 ? omega : 0;

        // Correct robot angle for gyro heading wander when no rotation stick is applied
        if (omega == 0.0 && !RobotContainer.ActiveOpMode.gamepad1.back) {
            if (m_pidDelay > 0) {
                m_pidDelay--;
            } else {
                // If heading target is unset, lock to current heading
                if (m_PIDTarget == null) {
                    m_PIDTarget = RobotContainer.gyro.getYawAngle();
                    m_headingPID.reset();
                }

                // Compute heading stabilization rotation
                omega = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYawAngle()));
            }
        } else {
            // Rotational input received or gyro reset pressed; release heading lock
            m_PIDTarget = null;
            m_pidDelay = 10;
        }

        // Calculate power factor with trigger boost
        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter);

        // Scale by maximum drivetrain linear velocity
        powerFactor = powerFactor * RobotContainer.drivesystem.MAX_SPEED;

        // Scale velocity components
        dX *= powerFactor;
        dY *= powerFactor;
        omega *= powerFactor;

        // Command field-oriented drive
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega);
    }

    /**
     * TeleOp drive command runs continuously until OpMode ends or another command takes over.
     *
     * @author gbuckholtz
     *
     * @return always false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Called when the command finishes or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
    }
}