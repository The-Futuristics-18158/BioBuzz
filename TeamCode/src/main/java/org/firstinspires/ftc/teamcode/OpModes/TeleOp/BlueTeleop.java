package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * TeleOp OpMode for the Blue Alliance.
 * Initializes the robot container for blue alliance and runs the periodic command scheduler loop.
 *
 * @author gbuckholtz
 */
@TeleOp(name = "Blue TeleOp", group = "OpMode")
public class BlueTeleop extends CommandOpMode {

    /**
     * Initializes all robot subsystems, configures TeleOp controls, sets alliance to blue,
     * and waits for match start.
     *
     * @author gbuckholtz
     */
    @Override
    public void initialize() {
        // Initialize robot for Blue Alliance (isRedAlliance = false)
        RobotContainer.Init(this, false);

        // Perform TeleOp-specific initialization
        RobotContainer.Init_TeleOp();

        // Wait for match play button press
        waitForStart();

        // Start TeleOp commands if OpMode is active
        if (opModeIsActive()) {
            // Schedule start-of-teleop commands if needed
        }
    }

    /**
     * Main execution loop called continuously while the OpMode is active.
     * Executes robot container periodic routines.
     *
     * @author gbuckholtz
     */
    @Override
    public void run() {
        // Run robot periodic processing
        RobotContainer.Periodic();
    }
}