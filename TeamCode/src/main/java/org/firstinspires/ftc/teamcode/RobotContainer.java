package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.GhostTeleOp;
import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.IntakeSequence;
import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDrive;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode.HuntModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.VacuumModeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.VacuumModeOn;
import org.firstinspires.ftc.teamcode.Commands.UptakeRampControle;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.RampCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.GoalTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.UptakeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.HoodTilt;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.RampLift;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShotBlock;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.PinpointOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

import java.util.List;

/** TODO
 * Adjust UptakeSubsystem to use one servo instead of two, and update all commands that use it    (no uptake)
 * Adjust Flywheel to use two motors instead of one, and update all commands that use it (in progress)
 * Remove Operator controls and references to it, or adjust to be used for something else         (completed)
 * Adjust shooting commands to not need to decide left vs right
 * Add constant tracking for the turret to the goal using the limelight
 * refactor ClimbSubsystem code to work with the DriveTrain
 *
 *
 * Update pinpointDriver.setOffsets() with correct values, line 48 of PinpointOdometry.java (completed)
 * Update Drive Kinematics with correct values, lines 31-32 of DriveTrain.java (completed)
 * Adjust ShotBlock subsystem (completed)
 * Adjust HoodTilt (completed)
 * Add the turret's subsystem (completed)
 * Remove code related to colour detection and pattern matching (completed)
    * Remove Obelisk.java and references to it                  (completed)
    * Remove code related to artifact colour detection          (completed)
        * Remove references to ColourSensor subsystem           (completed)
 */

public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // team alliance color = false if robot on blue alliance, true for red
    public static boolean isRedAlliance;

    // FTC dashboard and telemetries
    //public static Panels Panels;
    public static TelemetrySubsystem telemetrySubsystem;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;
    public static ElapsedTime exectimer;

    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static Gyro gyro;
    public static PinpointOdometry odometryPod;
    public static DriveTrain drivesystem;
    public static Odometry odometry;
    public static HoodTilt hoodtilt;
    public static ShotBlock shotblock;
    public static RampLift ramplift;
    public static Intake intake;
    public static Flywheel shooter;
    public static LimeLight limeLight;
    public static RampCamera rampCamera;
    public static UptakeSensor uptakeSensor;
    public static GoalTargeting targeting;
    public static Blinkin ledsubsystem;


    // Angle of the robot at the start of auto
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;
    public static boolean isVacuuming = false;
    public static boolean safeZone;

    // List of robot control and expansion hubs - used for caching of I/O
    static List<LynxModule> allHubs;

    // Robot Modes
    public enum Modes { Off, AutoInit, Auto, TeleOpInit, TeleOp}
    private static Modes CurrentRobotMode;

    public static double intervaltime;

    /**Robot initialization - common to both auto and teleop
     * @param mode A value from the Modes enum representing the current opmode being run, valid Modes as of 2/9/2026: Off, AutoInit, Auto, TeleOp
     * @param RedAlliance True if red alliance, false if blue alliance
     */
    public static void Init(CommandOpMode mode, boolean RedAlliance) {

        // save pointer to active OpMode
        ActiveOpMode = mode;

        // set alliance colour
        isRedAlliance = RedAlliance;

        // set robot mode - robot is off until we have initialized
        CurrentRobotMode = Modes.Off;

        // create list of robot control and expansion hubs
        // set each for manual caching - cache updated in periodic()
        allHubs = ActiveOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // create and reset timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        exectimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        // set up dashboard and various telemetries
        // Panels = new Panels();
        telemetrySubsystem = new TelemetrySubsystem();

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        odometryPod = new PinpointOdometry();
        drivesystem = new DriveTrain();
        odometry = new Odometry();
        hoodtilt = new HoodTilt();
        shotblock = new ShotBlock();
        ramplift = new RampLift();
        intake = new Intake();
        shooter = new Flywheel();
        limeLight = new LimeLight();
        rampCamera = new RampCamera("RampCam");
        uptakeSensor = new UptakeSensor();
        targeting = new GoalTargeting();
        ledsubsystem = new Blinkin();
    }

    /**Init teleop runs when you hit play*/
    public static void Init_TeleOp() {

        // robot is in teleop mode
        CurrentRobotMode = Modes.TeleOp;

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

//              -------------------------- (Driver) Odometry Reset --------------------------

        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> odometry.setCurrentPos
                (AutoFunctions.redVsBlue(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-90.0)))))));


//              -------------------------- (Driver) Shooting Controls  --------------------------

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new ShotSequence());

        driverOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(new VacuumModeOn(), new VacuumModeOff());

//              -------------------------- (Driver) Intake Systems --------------------------

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new ParallelCommandGroup(new IntakeCommand(), new UptakeRampControle()));

        Trigger mytrigger = new Trigger(()-> (RobotContainer.ActiveOpMode.gamepad1.left_trigger > 0.5));
        mytrigger.whileActiveOnce(new ReverseIntakeCommand());

        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new IntakeSequence());

        driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new HuntModeCommand());

        driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new GhostTeleOp());

//             -------------------------- Test Buttions/Manual Controls --------------------------

        // Block/Unblock
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->shotblock.Unblock()));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->shotblock.Block()));

        // Hood Adjust
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->hoodtilt.SetHoodPosition(hoodtilt.GetHoodPosition()-0.01)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(()->hoodtilt.SetHoodPosition(hoodtilt.GetHoodPosition()+0.01)));

//      -------------------------- (Driver) Turning To Exact Angle --------------------------
//        // Turn To 0 degrees
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new TurnTo(AutoFunctions.redVsBlue(0.0), false, 5.0));
//
//        // Turn To 90 degrees
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new TurnTo(AutoFunctions.redVsBlue(-90.0), false, 5.0));
//
//        // Turn To 180 degrees
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new TurnTo(AutoFunctions.redVsBlue(180.0), false, 5.0));
//
//        // Turn To 270 degrees
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TurnTo(AutoFunctions.redVsBlue(90.0), false, 5.0));


        //             -------------------------- Operator Controls --------------------------

        // enable/disable auto targeting
        toolOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(new InstantCommand(()->targeting.EnableAutoTargeting(!targeting.isAutoTargetingEnabled())));


        // OLD - not used - to be deleted
        // manual rotate turret
//        toolOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new SequentialCommandGroup(
//                new InstantCommand(()-> targeting.EnableAutoTargeting(false)),
//                new ManualRotateTurret(true)));
//        toolOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new SequentialCommandGroup(
//                new InstantCommand(()-> targeting.EnableAutoTargeting(false)),
//                new ManualRotateTurret(false)));


        //      -------------------------- Examples --------------------------
        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.

        // example turn to command
        // driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld((new TurnTo(AutoFunctions.redVsBlue(0.0), true, 3.0)));

        // example sequential command
        // driverOp.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ExampleCommandGroup());


        // example of binding more complex command to a button. This would be in a separate command file
        // driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ExampleCommand());

        // add other button commands here
        // Note: can trigger commands on
        // whenPressed - once when button is pressed
        // whenHeld - runs command while button held, but does not restart if command ends
        // whileHeld - runs command while button held, but will restart command if it ends
        // whenReleased - runs once when button is released
        // togglewhenPressed - turns command on and off at each button press

        // set limelight to apriltag pipeline
        limeLight.SetPipelineMode(0);
    }

    /**Robot starting code for teleop - This runs once at start of teleop*/
    public static void Start_TeleOp() {

        // robot is in teleop mode
        CurrentRobotMode = Modes.TeleOp;

    }

    /**Robot initialization for auto - This runs once at initialization of auto*/
    public static void Init_Auto() {

        // robot is in auto init mode
        CurrentRobotMode = Modes.AutoInit;

        // set limelight to apriltag pipeline
        limeLight.SetPipelineMode(0);
    }

    /**Robot starting code for auto - This runs once at start of auto*/
    public static void Start_Auto() {

        // robot is in auto mode
        CurrentRobotMode = Modes.Auto;

        // set limelight to apriltag pipeline
        limeLight.SetPipelineMode(0);

        // set default shooter speed control
        //shooter.setDefaultCommand(new DefaultShooterSpeed());
    }


    /**call this function periodically to operate scheduler*/
    public static void Periodic() {

        // clear I/O cache for robot control and expansion hubs
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // actual interval time
        intervaltime = timer.milliseconds();

        // execute robot periodic function 50 times per second (=50Hz)
        if (intervaltime>=20.0) {
            safeZone = RobotContainer.targeting.IsRobotInAllowableShotZone();

            // reset timer
            timer.reset();

            // start execution timer
            exectimer.reset();

            if (isVacuuming){
                if (!safeZone){
                    new VacuumModeOff().schedule();
                }
            }

            // run scheduler
            CommandScheduler.getInstance().run();

            // report robot odometry on robot controller
            telemetrySubsystem.odometryTelemetry();

            // report time interval on robot controller
            telemetrySubsystem.timerOdometry();

            //telemetrySubsystem.addData("Turret Target (degrees): ", turret.getTurretTargetDegrees());
            //telemetrySubsystem.addData("Turret Current (degrees): ", turret.getTurretDegrees());
            //telemetrySubsystem.addData("Intake Power: ", intake.readIntakeSetPower());
            //telemetrySubsystem.addData("Intake Motor Speed (rps): ", intake.intakeMotorCurrentSpeed());

            telemetrySubsystem.addData("in shot zone", safeZone);
            telemetrySubsystem.addData("isVacuuming", isVacuuming);

            telemetrySubsystem.update();
        }
    }


    /**Gets the current alliance colour
     * @return True if red alliance, false if blue alliance
     */
    public static boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**Returns the current robot mode
     * @return a value from the Modes enum, valid Modes as of 2/9/2026: Off, AutoInit, Auto, TeleOp*/
    public static Modes GetCurrentMode() { return CurrentRobotMode; }

    /**Gets our most commonly used starting angles for auto when we're on blue alliance
     * @return The angle of the robot when it's facing the red alliance drive team*/
    public static double getBlueStartAngle() {
        return BlueStartAngle;
    }

    /**Gets our most commonly used starting angles for auto when we're on red alliance
     * @return The angle of the robot when it's facing the blue alliance drive team*/
    public static double getRedStartAngle() {
        return RedStartAngle;
    }

}
