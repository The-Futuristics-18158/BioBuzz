package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem controlling the dual-motor shooter flywheel mechanism.
 * Utilizes closed-loop PIF (Proportional-Integral-Feedforward) control to maintain target rotational speed in RPM.
 *
 * @author superzokabear
 */
public class Flywheel extends SubsystemBase {

    // Dual flywheel motor hardware references
    private final DcMotorEx flywheelMotorRight;
    private final DcMotorEx flywheelMotorLeft;

    // Conversion and physical limits constants
    private final double MAXRPM = 6000.0;
    private final double TICKSPStoRPM = (1.0 / 28.0) * 60.0;

    // Flywheel target and actual velocity
    public static double TargetSpeed;
    private double CurrentSpeed;

    // PIF Controller Gains
    private final double FsGain = 0.0;
    private final double FvGain = 1.24 * 0.00016667; // Feedforward velocity gain based on 1.0 / 6000 RPM
    public final double PGain = 8.0 * 0.00016667;   // Proportional gain
    public final double IGain = 0.0002;             // Integral gain

    // Integrated error and loop timing
    private double IError;
    private ElapsedTime timer;

    /**
     * Initializes the flywheel subsystem, configures motor directions, sets zero-power behavior
     * to float (coast), and zeroes controller state variables.
     *
     * @author superzokabear
     */
    public Flywheel() {
        // Retrieve flywheel motors from the active OpMode's hardware map
        flywheelMotorRight = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        flywheelMotorLeft = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        // Both motors are reversed for proper shooter rotation
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to coast when unpowered
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initially zero the motor power
        flywheelMotorRight.setPower(0.0);
        flywheelMotorLeft.setPower(0.0);

        // Initialize error accumulator and timer
        timer = new ElapsedTime();
        timer.reset();
        IError = 0.0;

        // Target speed defaults to 0.0 RPM
        TargetSpeed = 0.0;
    }

    /**
     * Method called periodically by the command scheduler to compute and apply the PIF velocity control loop.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        // If robot is not in Auto or TeleOp, force flywheel motors off
        if (RobotContainer.GetCurrentMode() != RobotContainer.Modes.Auto &&
                RobotContainer.GetCurrentMode() != RobotContainer.Modes.TeleOp) {
            flywheelMotorRight.setPower(0.0);
            flywheelMotorLeft.setPower(0.0);
        } else {
            // Calculate current velocity in RPM from encoder ticks per second
            CurrentSpeed = flywheelMotorRight.getVelocity() * TICKSPStoRPM;

            // Calculate speed error in RPM
            double SpeedError = TargetSpeed - CurrentSpeed;

            // Determine time elapsed since last loop cycle
            double dt = timer.seconds();
            timer.reset();

            // Integrate speed error
            IError += IGain * SpeedError * 0.02;

            // Anti-windup protection to prevent overshoot
            if (SpeedError < -50.0 && IError > 0.0)
                IError *= 0.90;
            if (SpeedError > 50.0 && IError < 0.0)
                IError *= 0.90;

            // Clamp integrated error bounds
            if (IError > 0.15) IError = 0.15;
            if (IError < -0.1) IError = -0.1;

            // Compute PIF controller output power
            double NewPower = FsGain +                     // Static friction feedforward
                    FvGain * TargetSpeed +                 // Velocity feedforward
                    PGain * SpeedError +                   // Proportional feedback
                    IError;                                // Integral feedback

            // Only drive motor in positive direction, otherwise coast
            if (SpeedError < -50.0)
                NewPower = 0.0;
        }
    }

    /**
     * Sets the desired target speed for the shooter flywheel in RPM.
     *
     * @author superzokabear
     *
     * @param RPM the desired flywheel speed in revolutions per minute
     */
    public void SetFlywheelSpeed(double RPM) {
        // Update commanded flywheel speed
        TargetSpeed = RPM;
    }

    /**
     * Retrieves the current measured flywheel speed in RPM.
     *
     * @author superzokabear
     *
     * @return current measured flywheel speed in RPM
     */
    public double GetFlyWheelSpeed() {
        // Return current measured speed
        return CurrentSpeed;
    }

    /**
     * Retrieves the target flywheel speed in RPM.
     *
     * @author superzokabear
     *
     * @return target flywheel speed in RPM
     */
    public double GetFlyWheelTargetSpeed() {
        // Return target speed
        return TargetSpeed;
    }
}