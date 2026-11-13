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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.GoalTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.UptakeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.HoodTilt;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Kicker;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.PinpointOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

import java.util.List;

/**
 * RobotContainer manages the robot's hardware instances, subsystem instantiations,
 * gamepad mappings, alliance configurations, and periodic scheduler executions.
 *
 * @author gbuckholtz
 */
public class RobotContainer {

    // Active OpMode instance
    public static CommandOpMode ActiveOpMode;

    // Alliance configuration (true = Red Alliance, false = Blue Alliance)
    public static boolean isRedAlliance;

    // Telemetry subsystem
    public static TelemetrySubsystem telemetrySubsystem;

    // Timing tracking
    private static ElapsedTime timer;
    public static ElapsedTime exectimer;

    // Gamepad wrappers
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // Subsystems
    public static Gyro gyro;
    public static PinpointOdometry odometryPod;
    public static DriveTrain drivesystem;
    public static Odometry odometry;
    public static HoodTilt hoodtilt;
    public static Kicker kicker;
    public static Intake intake;
    public static Flywheel shooter;
    public static LimeLight limeLight;
    public static UptakeSensor uptakeSensor;
    public static GoalTargeting targeting;
    public static Blinkin ledsubsystem;

    // Starting angles for Autonomous
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;
    public static boolean isVacuuming = false;
    public static boolean safeZone;

    // Lynx expansion hubs for manual bulk caching
    static List<LynxModule> allHubs;

    // Robot operational modes
    public enum Modes { Off, AutoInit, Auto, TeleOpInit, TeleOp }
    private static Modes CurrentRobotMode;

    public static double intervaltime;

    /**
     * Initializes all robot hardware, subsystems, timers, and manual Lynx bulk caching.
     * Common initialization routine for both Autonomous and TeleOp OpModes.
     *
     * @author gbuckholtz
     *
     * @param mode active CommandOpMode instance
     * @param RedAlliance true if red alliance, false if blue alliance
     */
    public static void Init(CommandOpMode mode, boolean RedAlliance) {
        // Save pointer to active OpMode
        ActiveOpMode = mode;

        // Set alliance color
        isRedAlliance = RedAlliance;

        // Initialize mode to Off
        CurrentRobotMode = Modes.Off;

        // Set manual bulk caching mode for all Lynx hubs to optimize I/O polling
        allHubs = ActiveOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize timers
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        exectimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        exectimer.reset();

        // Initialize telemetry subsystem
        telemetrySubsystem = new TelemetrySubsystem();

        // Clear any lingering scheduled commands
        CommandScheduler.getInstance().cancelAll();

        // Instantiate gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // Instantiate subsystems
        gyro = new Gyro();
        odometryPod = new PinpointOdometry();
        drivesystem = new DriveTrain();
        odometry = new Odometry();
        hoodtilt = new HoodTilt();
        kicker = new Kicker();
        intake = new Intake();
        shooter = new Flywheel();
        limeLight = new LimeLight();
        uptakeSensor = new UptakeSensor();
        targeting = new GoalTargeting();
    }

    /**
     * Configures TeleOp-specific default commands and driver/operator button mappings.
     *
     * @author gbuckholtz
     */
    public static void Init_TeleOp() {
        // Set mode to TeleOp
        CurrentRobotMode = Modes.TeleOp;

        // Default drivetrain command: manual field-oriented drive
        drivesystem.setDefaultCommand(new ManualDrive());

        // (Driver) Odometry Reset: BACK button resets pose to alliance origin
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> odometry.setCurrentPos(
                AutoFunctions.redVsBlue(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-90.0)))))));

        // (Driver) Shooting Controls: RIGHT_BUMPER executes shooting sequence
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new ShotSequence());

        // (Driver) Vacuum Mode: Y button toggles vacuum mode on/off
        driverOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(new VacuumModeOn(), new VacuumModeOff());

        // (Driver) Intake Systems: LEFT_BUMPER runs intake and uptake control in parallel
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new ParallelCommandGroup(new IntakeCommand(), new UptakeRampControle()));

        // (Driver) Reverse Intake: LEFT_TRIGGER reverses intake
        Trigger leftTrigger = new Trigger(() -> (RobotContainer.ActiveOpMode.gamepad1.left_trigger > 0.5));
        leftTrigger.whileActiveOnce(new ReverseIntakeCommand());

        // (Driver) Intake Sequence: B button runs full intake sequence
        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new IntakeSequence());

        // (Driver) Hunt Mode: A button runs hunt mode
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new HuntModeCommand());

        // (Driver) Ghost TeleOp: X button runs autonomous routine
        driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new GhostTeleOp());

        // (Driver) Hood Manual Trim: DPAD LEFT / RIGHT increments hood angle
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> hoodtilt.SetHoodPosition(hoodtilt.GetHoodPosition() - 0.01)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> hoodtilt.SetHoodPosition(hoodtilt.GetHoodPosition() + 0.01)));

        // (Operator) Auto Targeting: START button toggles automated targeting
        toolOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> targeting.EnableAutoTargeting(!targeting.isAutoTargetingEnabled())));

        // Set Limelight to AprilTag pipeline (0)
        limeLight.SetPipelineMode(0);
    }

    /**
     * Called once when the TeleOp match period begins.
     *
     * @author gbuckholtz
     */
    public static void Start_TeleOp() {
        // Set mode to TeleOp
        CurrentRobotMode = Modes.TeleOp;
    }

    /**
     * Initializes robot configuration specifically for Autonomous mode.
     *
     * @author gbuckholtz
     */
    public static void Init_Auto() {
        // Set mode to AutoInit
        CurrentRobotMode = Modes.AutoInit;

        // Switch Limelight to AprilTag pipeline
        limeLight.SetPipelineMode(0);
    }

    /**
     * Called once when the Autonomous match period begins.
     *
     * @author gbuckholtz
     */
    public static void Start_Auto() {
        // Set mode to Auto
        CurrentRobotMode = Modes.Auto;

        // Switch Limelight to AprilTag pipeline
        limeLight.SetPipelineMode(0);
    }

    /**
     * Main periodic update loop. Clears Lynx hub bulk cache, runs the FTCLib CommandScheduler
     * at 50 Hz (20ms interval), evaluates shot zones, and updates telemetry.
     *
     * @author gbuckholtz
     */
    public static void Periodic() {
        // Clear bulk caching on all Lynx hardware modules
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Measure loop interval
        intervaltime = timer.milliseconds();

        // Execute scheduled routines at 50 Hz
        if (intervaltime >= 20.0) {
            safeZone = RobotContainer.targeting.IsRobotInAllowableShotZone();

            // Reset loop timers
            timer.reset();
            exectimer.reset();

            // Deactivate vacuum mode if outside safe zone
            if (isVacuuming && !safeZone) {
                new VacuumModeOff().schedule();
            }

            // Run command scheduler
            CommandScheduler.getInstance().run();

            // Update odometry and loop timing telemetry
            telemetrySubsystem.odometryTelemetry();
            telemetrySubsystem.timerOdometry();
            telemetrySubsystem.addData("in shot zone", safeZone);
            telemetrySubsystem.addData("isVacuuming", isVacuuming);
            telemetrySubsystem.update();
        }
    }

    /**
     * Checks whether the robot is currently configured for the Red Alliance.
     *
     * @author gbuckholtz
     *
     * @return true if Red Alliance, false if Blue Alliance
     */
    public static boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Retrieves the current robot operational mode.
     *
     * @author gbuckholtz
     *
     * @return current Modes enum value (Off, AutoInit, Auto, TeleOpInit, TeleOp)
     */
    public static Modes GetCurrentMode() {
        return CurrentRobotMode;
    }

    /**
     * Retrieves the standard starting orientation angle for the Blue Alliance in Autonomous.
     *
     * @author gbuckholtz
     *
     * @return blue alliance starting angle in degrees (-90.0)
     */
    public static double getBlueStartAngle() {
        return BlueStartAngle;
    }

    /**
     * Retrieves the standard starting orientation angle for the Red Alliance in Autonomous.
     *
     * @author gbuckholtz
     *
     * @return red alliance starting angle in degrees (90.0)
     */
    public static double getRedStartAngle() {
        return RedStartAngle;
    }
}