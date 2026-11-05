package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Shooter Flywheel Subsystem
 *
 * @author superzokabear
 */
//@Configurable
public class Flywheel extends SubsystemBase {

    // Local objects and variables here
    private final DcMotorEx flywheelMotorRight;
    private final DcMotorEx flywheelMotorLeft;


    // constants
    private final double MAXRPM = 6000.0;
    private final double TICKSPStoRPM = (1/28.0)*60.0;

    // target speed
    public static double TargetSpeed;  // When disabling dashboard/panels turn back to privet. Make static when not using pannels
    private double CurrentSpeed;

    // PIF Controller Gains
    private final double FsGain = 0.0;
    private final double FvGain = 1.24*0.00016667; //initial value=1.0/6000rpm=0.00016667
    public final double PGain = 8.0*0.00016667;
    public final double IGain = 0.0002;


    // integrated error
    private double IError;
    private ElapsedTime timer;

    //private boolean FlywheelTrackingOn = true;

    /** Place code here to initialize subsystem */
    public Flywheel() {
        // create motor
        flywheelMotorRight = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        flywheelMotorLeft = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        // one of the motors is reversed
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // important! - set motor to coast mode - only works for 0 power
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // motor is initially off
        flywheelMotorRight.setPower(0.0);
        flywheelMotorLeft.setPower(0.0);

        // reset integrated error
        timer = new ElapsedTime();
        timer.reset();
        IError=0.0;

        // reset target speed (rpm)
        TargetSpeed=0.0;


    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // if robot is not in auto or teleop, then force flywheel to be unpowered
        if (RobotContainer.GetCurrentMode()!= RobotContainer.Modes.Auto &&
            RobotContainer.GetCurrentMode()!= RobotContainer.Modes.TeleOp)
        {
            flywheelMotorRight.setPower(0.0);
            flywheelMotorLeft.setPower(0.0);
        }
        else {

            // only perform flywheel motor control if in teleop or auto

            //if (FlywheelTrackingOn){

            // our current speed
            CurrentSpeed = flywheelMotorRight.getVelocity() * TICKSPStoRPM;

            // our current speed error (in RPM)
            double SpeedError = TargetSpeed - CurrentSpeed;

            // integrated error
            // determine time since last iteration
            double dt = timer.seconds();
            timer.reset();
            // integrate speed error
            IError += IGain * SpeedError * 0.02;
            // anti-windup to prevent overshoots
            if (SpeedError < -50.0 && IError > 0.0)
                IError *= 0.90;
            if (SpeedError > 50.0 && IError < 0.0)
                IError *= 0.90;
            // integrated error limiter
            if (IError > 0.15) IError = 0.15;
            if (IError < -0.1) IError = -0.1;

            // PIF controller
            double NewPower = FsGain +          // static feedforward
                    FvGain * TargetSpeed +      // speed feedforward
                    PGain * SpeedError +        // proportional gain
                    IError;                     // integrated error


            // only drive motor in positive direction, otherwise let it coast
            if (SpeedError < -50.0)
                NewPower = 0.0;

            // Commented out these two lines at the start of BioBuzz to stop idle spinning
            //flywheelMotorRight.setPower(NewPower);
            //flywheelMotorLeft.setPower(NewPower);

            //PanelsTelemetry.INSTANCE.getTelemetry().addData("FlywheelSpeed", CurrentSpeed);
            //PanelsTelemetry.INSTANCE.getTelemetry().addData("FlywheelTarget", TargetSpeed);
            //PanelsTelemetry.INSTANCE.getTelemetry().addData("FlywheelPower", NewPower);
        }

    }

    // Place special subsystem methods here

    /** Sets shooter flywheel speed in rpm
     * @param RPM a double representing the desired flywheel speed in rpm. Negative values will be treated as 0.0.
     */
    public void SetFlywheelSpeed(double RPM){
        // Setting velocity using the RPMToVelocity methode
        TargetSpeed = RPM;

    }

    /**gets current flywheel speed in rpm
     * @return current flywheel speed in rpm
     */
    public double GetFlyWheelSpeed() {
        return CurrentSpeed;
    }

    /**gets target flywheel speed in rpm
     * @return target flywheel speed in rpm
     */
    public double GetFlyWheelTargetSpeed() {
        return TargetSpeed;
    }

//    public boolean GetSpeedTrackingOn(){
//
//        return FlywheelTrackingOn;
//    }

//    public void ToggleTrackng(){
//        // same as an if staitment
//        FlywheelTrackingOn = !FlywheelTrackingOn;
//    }

}