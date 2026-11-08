package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Setting up the intake subsystem.<p>
 * This makes the robot know that the intake motor exists.
 * This motor should run continuously not to a specific position.
 * Therefor this subsystem only has start and stop functions.
 * @author superzokabear
 */
public class Intake extends SubsystemBase {

    // Local objects and variables here
    private final DcMotorEx intakeMotor;

    // constants
    private final double TICKS_PER_ROTATION = 28.0;
    private final double INV_TICKS_PER_ROTATION = 1.0 / TICKS_PER_ROTATION;
    private final double GEAR_REDUCTION = 5.0 * 1.2; // Motor gearbox is 5:1 and the gearing is 1.2x.
    private final double TICKS_PER_INTAKE_ROTATION = TICKS_PER_ROTATION * GEAR_REDUCTION;


    // f and p gain units are in power/motor_rps
    // ideal no-load Fgain = 1.0 / 100rps = 0.01
    // June 2/2026 KN: Intake testing shows actual speed of 90rps for 100% power
    // therefore select Fgain = 0.01 * 100/90 = 0.0111
    private final double fgain = 0.0111;
    private final double pgain = 0.013;
    private final double igain = 0.001;

    // current motor values
    private double TargetSpeed;     // in motor rps
    private double CurrentSpeed;    // in motor rps
    private double CurrentPower;    // applied voltage -1.0 to 1.0

    // motor controller state values
    private double ierror;

    // agitator
    private boolean AgitateMode;
    private int AgitatorCount;

    /** Place code here to initialize subsystem */
    public Intake() {
        // Creates the motor using the hardware map
        intakeMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        // Sets motor direction
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Set motor power
        intakeMotor.setPower(0.0);
        // by default, set target speed to 0
        TargetSpeed = 0.0;
        // reset PIF controller
        ierror = 0.0;

        AgitatorCount = 0;
        AgitateMode = false;

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // counter used to generate on/off duty cycle of agitator
        AgitatorCount++;
        if (AgitatorCount > 10)
            AgitatorCount = 0;


        // if robot is not in auto or teleop, then force flywheel to be unpowered
        if (RobotContainer.GetCurrentMode()!= RobotContainer.Modes.Auto &&
                RobotContainer.GetCurrentMode()!= RobotContainer.Modes.TeleOp)
            intakeMotor.setPower(0.0);

        else {

            // do we revise target (Agitate) or leave as-is?
            double revisedtarget;
            if (AgitateMode && AgitatorCount < 2)
                revisedtarget = 0;
            else
                revisedtarget = TargetSpeed;

            // sets motor speed in rps (=28xticks/s)
            CurrentSpeed = intakeMotor.getVelocity() * INV_TICKS_PER_ROTATION;
            double error = revisedtarget - CurrentSpeed;//TargetSpeed - CurrentSpeed;

            // limit P and I action for large errors
            // to reduce control transients
            // only apply when not agitation)
            if (!AgitateMode && error > 20) error = 20;
            if (!AgitateMode && error < -20) error = -20;

            // integrated error
            ierror += igain * error;

            // anti-windup limiter
            if (ierror > 0.2) ierror = 0.2;
            if (ierror < -0.2) ierror = -0.2;

            // set intake motor power (PIF controller)
            CurrentPower = fgain * revisedtarget + pgain * error + ierror;
            intakeMotor.setPower(CurrentPower);

            //PanelsTelemetry.INSTANCE.getTelemetry().addData("IntakeSpeed", CurrentSpeed);
            //PanelsTelemetry.INSTANCE.getTelemetry().addData("IntakeTarget", revisedtarget);
            //PanelsTelemetry.INSTANCE.getTelemetry().addData("IntakePower", CurrentPower);
            //PanelsTelemetry.INSTANCE.getTelemetry().update();
        }
    }

    // Place special subsystem methods here

    /** Sets speed of intake in motor rps */
    public void intakeSetSpeed(double speed) { intakeSetSpeed(speed, false);}
    public void intakeSetSpeed(double speed, boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = speed;
    }


    /**Run the intake at set speed (rps)*/
    public void intakeRun(){ intakeRun(false);}
    public void intakeRun(boolean Agitate){
        AgitateMode = Agitate;
        TargetSpeed = 50.0;
    }

    /**Run the intake at set speed (rps)*/
    public void intakeRunReducedSpeed() { intakeRunReducedSpeed(false);}
    public void intakeRunReducedSpeed(boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = 30.0; // was 60.0
    }

    /**Run the intake at set speed (rps)*/
    public void intakeReverse(){intakeReverse(false);}
    public void intakeReverse(boolean Agitate) {
        AgitateMode = Agitate;
        TargetSpeed = -30.0;
    }

    /**Stop intake*/
    public void intakeStop(){
        ResetController();
        AgitateMode = false;
        TargetSpeed = 0.0;
    }

    /** Internal function that resets motor PIF controller */
    private void ResetController() { ierror = 0.0;}

    /** returns motor position in encoder ticks */
    public double GetMotorPostion() { return intakeMotor.getCurrentPosition();}


    /** Gets intake speed
     * @return intake speed in rps*/
    public double GetIntakeMotorCurrentSpeed () {
        return CurrentSpeed / GEAR_REDUCTION;
    }

    /** Sets intake speed (as opposed to at the motor shaft)
     @param speed - intake speed in rps*/
    public void SetIntakeMotorSpeed(double speed) {
        intakeSetSpeed(speed * GEAR_REDUCTION);
    }

}