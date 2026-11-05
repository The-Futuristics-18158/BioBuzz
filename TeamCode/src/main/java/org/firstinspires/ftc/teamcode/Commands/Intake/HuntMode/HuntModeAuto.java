package org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Intake.EndWhenFull.EndAfterTimeElapsed;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class HuntModeAuto extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    PIDController omegaControl;
    PIDController xControl;
    int finishedCounter;
    private ElapsedTime timer;
    private double seconds;
    private double targetAngle;

    // constructor
    public HuntModeAuto(){
        this(5.0);
    }
    public HuntModeAuto(double time) {
        haveArtifact = false;

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.intake);

        omegaControl = new PIDController(0.05, 0.0005, 0.01);
        xControl = new PIDController(0.0075, 0.0, 0.0);
        timer = new ElapsedTime();
        seconds = time;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        omegaControl.reset();
        xControl.reset();
        //finished = false;
        finishedCounter=0;
        timer.reset();
        if(RobotContainer.isRedAlliance()){
            targetAngle = 90.0;
        }else{
            targetAngle = -90.0;
        }
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        if (timer.seconds() >= seconds){ //May need to adjust timeout

            RobotContainer.intake.intakeStop();
            finishedCounter++;
            //finished = true;

        }else {
            finishedCounter=0;
            RobotContainer.intake.intakeRun();
        }

        double angleError = Utils.AngleDifference(RobotContainer.gyro.getYawAngle(),targetAngle);
        omega_speed = -omegaControl.calculate(angleError);

        List<ColorBlobLocatorProcessor.Blob> blobs;
        // change this later if for get green blob detections and or get purple blob detections (if we want to look for one specific color)
        //blobs = RobotContainer.rampCamera.GetAllBlobDetections();

        // If the robot sees an artifact in the camera
//        if (blobs != null && !(blobs.isEmpty())) {
//            haveArtifact = true;
//            blobX = blobs.get(0).getCircle().getCenter().x - 160.0;
//
//            // set forward speed to value depending on how far artifact from center of camera
//            // first number is forward
//            // second number is how quickly the speed goes down when artifact is off center
//            y_speed = 0.9 - 0.8*Math.min(Math.abs(blobX/160.0),1.0);
//
//            if (y_speed < 0.0){ y_speed = 0.0;}
//
//            if(!RobotContainer.isRedAlliance()){
//               y_speed = y_speed* -1.0;
//            }
//
//
//            if(RobotContainer.isRedAlliance()){
//                x_speed = -xControl.calculate(blobX);
//            }else{
//                x_speed = xControl.calculate(blobX);
//            }
//
//        // Don't see anything therefor go ahead, no strafe
//        } else {
//            haveArtifact = false;
//            blobX = 0;
//            //omega_speed = 0.0;
//            y_speed = 0.9;
//            x_speed = 0.0;
//            if(!RobotContainer.isRedAlliance()){
//               y_speed = y_speed* -1.0;
//            }
//        }

//       // if busy intaking robot, stop and wait for it to finish before proceeding
//        if (RobotContainer.distance.isRampArtifactPresent()) {
//            y_speed = 0.0;
//            x_speed = 0.0;
//        }

//        RobotContainer.drivesystem.FieldDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return (finishedCounter>=2);

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.drivesystem.RobotDrive(0,0,0);
        RobotContainer.intake.intakeStop();

    }

}