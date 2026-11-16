package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command that waits for the shooter flywheel to reach its target RPM or until a maximum spin-up timeout elapses.
 * <p>
 * <b>Teaching Concept - Conditional Wait with Fail-Safe Timeout:</b>
 * In autonomous and automated shooting sequences, we must wait for flywheels to reach
 * steady-state velocity before feeding balls. However, waiting indefinitely can freeze the robot
 * if battery voltage is low or a motor stalls. This command demonstrates using a boolean check
 * ({@link org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel#isAtTargetSpeed(double)}) combined
 * with an elapsed time fail-safe.
 *
 * @author knutt5, gbuckholtz
 */
public class WaitForSpinup extends CommandBase {

    private final double maxTimeoutSeconds;
    private final double toleranceRPM;
    private final ElapsedTime timer;

    /**
     * Constructs a WaitForSpinup command with default timeout (1.5 seconds) and tolerance (100 RPM).
     *
     * @author knutt5
     */
    public WaitForSpinup() {
        this(1.5, 100.0);
    }

    /**
     * Constructs a WaitForSpinup command with a custom timeout.
     *
     * @param timeoutSeconds maximum duration to wait before proceeding
     */
    public WaitForSpinup(double timeoutSeconds) {
        this(timeoutSeconds, 100.0);
    }

    /**
     * Constructs a WaitForSpinup command with custom timeout and speed tolerance.
     *
     * @param timeoutSeconds maximum duration to wait in seconds
     * @param toleranceRPM maximum speed error in RPM considered "ready"
     */
    public WaitForSpinup(double timeoutSeconds, double toleranceRPM) {
        this.maxTimeoutSeconds = timeoutSeconds;
        this.toleranceRPM = toleranceRPM;
        this.timer = new ElapsedTime();
    }

    /**
     * Called once when the command begins; resets the elapsed timer.
     *
     * @author knutt5
     */
    @Override
    public void initialize() {
        timer.reset();
    }

    /**
     * Periodically called while waiting for spinup.
     *
     * @author knutt5
     */
    @Override
    public void execute() {
        RobotContainer.telemetrySubsystem.addData("Spinup Wait (s)",
                String.format(java.util.Locale.US, "%.2f", timer.seconds()));
        RobotContainer.telemetrySubsystem.addData("Flywheel At Speed",
                RobotContainer.shooter.isAtTargetSpeed(toleranceRPM));
    }

    /**
     * Determines whether the shooter flywheel has reached target speed or timed out.
     *
     * @author knutt5
     *
     * @return true when target RPM is reached or fail-safe timeout elapses
     */
    @Override
    public boolean isFinished() {
        // Ready if flywheel is at target speed or timeout elapsed
        return RobotContainer.shooter.isAtTargetSpeed(toleranceRPM) || timer.seconds() >= maxTimeoutSeconds;
    }

    /**
     * Called when the command ends or is interrupted.
     *
     * @author knutt5
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        // Optional cleanup or telemetry logging
    }
}
