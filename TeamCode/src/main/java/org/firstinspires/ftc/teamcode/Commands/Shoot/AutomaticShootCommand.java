package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.FlywheelGap;

/**
 * Command that orchestrates the intelligent shooting sequence:
 * <ol>
 *   <li><b>Ball Detection:</b> Checks the top REV Color Sensor V3 to confirm a ball is present.</li>
 *   <li><b>Flywheel Sizing & Spin-up:</b> Reads ball color/type. Sets top servo to narrower gap for Pollen,
 *       or safely defaults to wider gap for Nectar. Spins up the dual flywheel motors to target RPM.</li>
 *   <li><b>Wait for Readiness:</b> Waits until the flywheels reach steady-state RPM and the servo settles.</li>
 *   <li><b>Kicker Advance:</b> Advances the kicker paddles to propel the ball into the flywheels.</li>
 *   <li><b>Clear Feed Path:</b> Rotates kicker paddles until side distance sensors confirm the fins
 *       have cleared the feed corridor for subsequent balls.</li>
 * </ol>
 *
 * @author gbuckholtz
 */
public class AutomaticShootCommand extends CommandBase {

    /**
     * Internal execution states of the automated shooting sequence.
     */
    private enum ShootState {
        WAIT_FOR_READY,
        ADVANCE_KICKER,
        CLEAR_FINS,
        COMPLETED
    }

    private ShootState state = ShootState.WAIT_FOR_READY;
    private final ElapsedTime stateTimer;

    // Configurable shooting parameters
    private final double targetRPM;
    private final double rpmTolerance;
    private final double spinupTimeoutSeconds;
    private final double kickDurationSeconds = 0.40;
    private final double servoTransitionDelaySeconds = 0.15;

    private BallColorSensor.BallType detectedBall = BallColorSensor.BallType.NONE;

    /**
     * Constructs an AutomaticShootCommand with default target RPM (4500 RPM).
     *
     * @author gbuckholtz
     */
    public AutomaticShootCommand() {
        this(4500.0, 100.0, 2.0);
    }

    /**
     * Constructs an AutomaticShootCommand with custom shooting parameters.
     *
     * @author gbuckholtz
     *
     * @param targetRPM desired flywheel velocity in revolutions per minute
     * @param rpmTolerance allowable error margin in RPM
     * @param spinupTimeoutSeconds maximum duration in seconds to wait for spin-up
     */
    public AutomaticShootCommand(double targetRPM, double rpmTolerance, double spinupTimeoutSeconds) {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.flywheelGap);
        addRequirements(RobotContainer.kicker);

        this.targetRPM = targetRPM;
        this.rpmTolerance = rpmTolerance;
        this.spinupTimeoutSeconds = spinupTimeoutSeconds;
        this.stateTimer = new ElapsedTime();
    }

    /**
     * Called once when the command is scheduled.
     * Evaluates ball presence and initiates the spinup and flywheel sizing sequence.
     * If no ball is detected, the command completes immediately to avoid dry-firing.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        stateTimer.reset();

        // 1. Check if a ball is present under the top sensor
        if (!RobotContainer.ballSensor.isBallPresent()) {
            // No ball detected - skip shooting to avoid dry-firing
            RobotContainer.telemetrySubsystem.addData("Shoot Aborted", "No ball detected in chamber");
            state = ShootState.COMPLETED;
            return;
        }

        // 2. Classify ball type (Nectar vs. Pollen), defaulting to Nectar (wider gap) for safety
        detectedBall = RobotContainer.ballSensor.getBallType();

        // 3. Set top servo for flywheel gap
        RobotContainer.flywheelGap.setGapForBall(detectedBall);

        // 4. Spin up flywheel to target speed
        RobotContainer.shooter.SetFlywheelSpeed(targetRPM);

        state = ShootState.WAIT_FOR_READY;
        stateTimer.reset();
    }

    /**
     * Periodically updates state transitions through spin-up, kicking, and fin clearing.
     *
     * @author gbuckholtz
     */
    @Override
    public void execute() {
        switch (state) {
            case WAIT_FOR_READY:
                // Wait for:
                // a) Flywheel to reach target RPM (or spinup timeout)
                // b) Top servo to settle in position
                boolean flywheelReady = RobotContainer.shooter.isAtTargetSpeed(rpmTolerance);
                boolean spinupTimedOut = stateTimer.seconds() >= spinupTimeoutSeconds;
                boolean servoSettled = stateTimer.seconds() >= servoTransitionDelaySeconds;

                if ((flywheelReady || spinupTimedOut) && servoSettled) {
                    RobotContainer.kicker.advanceToShoot();
                    state = ShootState.ADVANCE_KICKER;
                    stateTimer.reset();
                }
                break;

            case ADVANCE_KICKER:
                if (stateTimer.seconds() >= kickDurationSeconds) {
                    // Ball kicked: now rotate paddles until fins clear the feed path
                    RobotContainer.kicker.clearFeedPath();
                    state = ShootState.CLEAR_FINS;
                    stateTimer.reset();
                }
                break;

            case CLEAR_FINS:
                // Wait until the two side distance sensors detect the fins are clear (or 1s timeout)
                if (RobotContainer.kicker.isFeedPathClear() || stateTimer.seconds() >= 1.0) {
                    RobotContainer.kicker.stop();
                    state = ShootState.COMPLETED;
                }
                break;

            case COMPLETED:
            default:
                break;
        }

        // Telemetry update
        RobotContainer.telemetrySubsystem.addData("Shoot State", state.name());
        RobotContainer.telemetrySubsystem.addData("Detected Ball", detectedBall.name());
    }

    /**
     * Finishes when the complete shooting cycle is done.
     *
     * @author gbuckholtz
     *
     * @return true when shooting and fin clearing are completed
     */
    @Override
    public boolean isFinished() {
        return state == ShootState.COMPLETED;
    }

    /**
     * Called when the command ends normally or is interrupted.
     * Ensures kickers are stopped and resets the flywheel / top servo to default.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by another command
     */
    @Override
    public void end(boolean interrupted) {
        RobotContainer.kicker.stop();
        RobotContainer.shooter.SetFlywheelSpeed(0.0);
        RobotContainer.flywheelGap.setGap(FlywheelGap.GAP_DEFAULT);
    }
}
