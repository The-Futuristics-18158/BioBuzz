package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem controlling the dual-motor shooter flywheel mechanism.
 * Utilizes closed-loop PIF (Proportional-Integral-Feedforward) control to maintain target rotational speed in RPM.
 *
 * @author superzokabear
 */
public class SimplifiedFlywheel extends SubsystemBase {

    // Dual flywheel motor hardware references
    private final DcMotorEx flywheelMotorRight;
    private final DcMotorEx flywheelMotorLeft;

    // Conversion and physical limits constants
    private final double MAXRPM = 6000.0;
    private final double TICKSPStoRPM = (1.0 / 28.0) * 60.0;

    // Flywheel target and actual velocity
    public static double targetSpeed;
    private double CurrentSpeed;
    int ticksPerRev = 28; // as this motor’s spec has 28 pulses (ticks) per rotation

    /**
     * Initializes the flywheel subsystem, configures motor directions, sets zero-power behavior
     * to float (coast), and zeroes controller state variables.
     *
     * @author superzokabear
     */
    public SimplifiedFlywheel() {
        // Retrieve flywheel motors from the active OpMode's hardware map
        flywheelMotorRight = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        flywheelMotorLeft = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        // Both motors are reversed for proper shooter rotation
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to coast when unpowered
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initially zero the motor power
        flywheelMotorRight.setPower(0.0);
        flywheelMotorLeft.setPower(0.0);

        // Target speed defaults to 0.0 RPM
        targetSpeed = 0.0;

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
            flywheelMotorRight.setVelocity((targetSpeed / 60) * ticksPerRev);
            RobotContainer.telemetrySubsystem.addData("RPM Right", CurrentSpeed);
            //left
            CurrentSpeed = flywheelMotorLeft.getVelocity() * TICKSPStoRPM;
            flywheelMotorLeft.setVelocity((targetSpeed / 60) * ticksPerRev);
            RobotContainer.telemetrySubsystem.addData("RPM Left", CurrentSpeed);
            RobotContainer.telemetrySubsystem.addData("Target RPM", targetSpeed);

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
        targetSpeed = RPM;
    }
public void inkFlywheelSpeed(double newSpeed){
        targetSpeed=targetSpeed+newSpeed;
}
    public void decFlywheelSpeed(double newSpeed){
        targetSpeed=targetSpeed-newSpeed;
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
        return targetSpeed;
    }
}