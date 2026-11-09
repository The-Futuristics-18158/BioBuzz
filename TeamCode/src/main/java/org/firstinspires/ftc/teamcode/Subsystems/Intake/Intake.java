package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem controlling the intake roller mechanism.
 * Uses a closed-loop PIF speed controller with optional agitation mode to prevent artifact jams.
 *
 * @author superzokabear
 */
public class Intake extends SubsystemBase {

    // Intake motor hardware reference
    private final DcMotorEx intakeMotor;

    private final Servo leftServo;
    private final Servo rightServo;

    private final double SERVO_ON = 1;
    private final double SERVO_OFF = 0.5;

    // Gear reduction and encoder ticks constants
    private final double TICKS_PER_ROTATION = 28.0;
    private final double INV_TICKS_PER_ROTATION = 1.0 / TICKS_PER_ROTATION;
    private final double GEAR_REDUCTION = 5.0 * 1.2; // Motor gearbox is 5:1 and external gearing is 1.2x
    private final double TICKS_PER_INTAKE_ROTATION = TICKS_PER_ROTATION * GEAR_REDUCTION;

    // PIF controller gains (power per motor RPS)
    private final double fgain = 0.0111;
    private final double pgain = 0.013;
    private final double igain = 0.001;

    // Current motor state values
    private double TargetSpeed;     // Target speed in motor RPS
    private double CurrentSpeed;    // Measured speed in motor RPS
    private double CurrentPower;    // Applied motor power (-1.0 to 1.0)

    // Integral error accumulator
    private double ierror;

    // Agitation state
    private boolean AgitateMode;
    private int AgitatorCount;

    /**
     * Initializes the intake subsystem, configures motor direction and zero power behavior,
     * and resets controller errors and agitation states.
     *
     * @author superzokabear
     */
    public Intake() {
        // Retrieve the intake motor from the active OpMode's hardware map
        intakeMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Configure motor direction and float zero-power behavior
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initially zero the motor power and target speed
        intakeMotor.setPower(0.0);
        TargetSpeed = 0.0;

        // Reset PIF controller integrated error
        ierror = 0.0;

        // Initialize agitation mode
        AgitatorCount = 0;
        AgitateMode = false;

        //Creates the servos for extracting from the flower
        leftServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftFlowerExtractor");
        rightServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightFlowerExtractor");
        leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void servoOn() {
        //Turns servos on
        leftServo.setPosition(SERVO_ON);

        rightServo.setPosition(SERVO_ON);
    }

    public void servoOff() {
        //Turns servos off

        leftServo.setPosition(SERVO_OFF);

        rightServo.setPosition(SERVO_OFF);
    }

    /**
     * Method called periodically by the command scheduler to compute and apply the intake motor velocity PIF loop.
     *
     * @author superzokabear
     */
    @Override
    public void periodic() {
        // Counter used to generate on/off duty cycle for agitation
        AgitatorCount++;
        if (AgitatorCount > 10)
            AgitatorCount = 0;

        // If robot is not in auto or teleop, force motor off
        if (RobotContainer.GetCurrentMode() != RobotContainer.Modes.Auto &&
                RobotContainer.GetCurrentMode() != RobotContainer.Modes.TeleOp) {
            intakeMotor.setPower(0.0);
        } else {
            // Apply agitation pulsing if active
            double revisedtarget;
            if (AgitateMode && AgitatorCount < 2)
                revisedtarget = 0;
            else
                revisedtarget = TargetSpeed;

            // Compute current speed in motor revolutions per second
            CurrentSpeed = intakeMotor.getVelocity() * INV_TICKS_PER_ROTATION;
            double error = revisedtarget - CurrentSpeed;

            // Limit P and I action for large errors to reduce transients
            if (!AgitateMode && error > 20) error = 20;
            if (!AgitateMode && error < -20) error = -20;

            // Integrate error
            ierror += igain * error;

            // Anti-windup limiter
            if (ierror > 0.2) ierror = 0.2;
            if (ierror < -0.2) ierror = -0.2;

            // Compute PIF controller output power and set motor power
            CurrentPower = fgain * revisedtarget + pgain * error + ierror;
            intakeMotor.setPower(CurrentPower);
        }
    }

    /**
     * Sets the target speed of the intake in motor RPS without agitation.
     *
     * @author superzokabear
     *
     * @param speed target speed in motor revolutions per second
     */
    public void intakeSetSpeed(double speed) {
        intakeSetSpeed(speed, false);
    }

    /**
     * Sets the target speed of the intake in motor RPS with optional agitation.
     *
     * @author superzokabear
     *
     * @param speed target speed in motor revolutions per second
     * @param Agitate true to enable intermittent agitation pulsing, false for continuous
     */
    public void intakeSetSpeed(double speed, boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = speed;
    }

    /**
     * Runs the intake forward at standard collection speed (90 motor RPS) without agitation.
     *
     * @author superzokabear
     */
    public void intakeRun() {
        intakeRun(false);
    }

    /**
     * Runs the intake forward at standard collection speed (90 motor RPS) with optional agitation.
     *
     * @author superzokabear
     *
     * @param Agitate true to enable agitation, false otherwise
     */
    public void intakeRun(boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = 50.0;
        servoOn();
    }

    /**
     * Runs the intake forward at reduced speed (75 motor RPS) without agitation.
     *
     * @author superzokabear
     */
    public void intakeRunReducedSpeed() {
        intakeRunReducedSpeed(false);
    }

    /**
     * Runs the intake forward at reduced speed (75 motor RPS) with optional agitation.
     *
     * @author superzokabear
     *
     * @param Agitate true to enable agitation, false otherwise
     */
    public void intakeRunReducedSpeed(boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = 30.0;
    }

    /**
     * Runs the intake in reverse (-30 motor RPS) to clear jams without agitation.
     *
     * @author superzokabear
     */
    public void intakeReverse() {
        intakeReverse(false);
    }

    /**
     * Runs the intake in reverse (-30 motor RPS) with optional agitation.
     *
     * @author superzokabear
     *
     * @param Agitate true to enable agitation, false otherwise
     */
    public void intakeReverse(boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = -30.0;
    }

    /**
     * Stops the intake motor and resets the PIF controller state.
     *
     * @author superzokabear
     */
    public void intakeStop() {
        ResetController();
        AgitateMode = false;
        TargetSpeed = 0.0;
        servoOff();
    }

    /**
     * Resets the PIF controller integrated error accumulator.
     *
     * @author superzokabear
     */
    private void ResetController() {
        ierror = 0.0;
    }

    /**
     * Returns the current intake motor position in encoder ticks.
     *
     * @author superzokabear
     *
     * @return current motor encoder position in ticks
     */
    public double GetMotorPostion() {
        return intakeMotor.getCurrentPosition();
    }

    /**
     * Retrieves the measured rotational speed at the intake roller shaft in RPS.
     *
     * @author superzokabear
     *
     * @return intake roller speed in revolutions per second
     */
    public double GetIntakeMotorCurrentSpeed() {
        return CurrentSpeed / GEAR_REDUCTION;
    }

    /**
     * Sets the target speed at the intake roller shaft in RPS.
     *
     * @author superzokabear
     *
     * @param speed desired intake roller speed in revolutions per second
     */
    public void SetIntakeMotorSpeed(double speed) {
        intakeSetSpeed(speed * GEAR_REDUCTION);
    }
}