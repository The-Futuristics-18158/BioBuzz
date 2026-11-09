package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to manage telemetry outputs to the FTC Driver Station.
 * Supports dual modes: detailed diagnostic telemetry for testing and clean, concise telemetry for operators.
 *
 * @author Kw126
 */
public class TelemetrySubsystem extends SubsystemBase {

    // Local objects and variables here
    public static Telemetry RCTelemetry;
    public static boolean testingTelemetry = true;

    /**
     * Initializes the telemetry subsystem using the active OpMode's telemetry handle.
     *
     * @author Kw126
     */
    public TelemetrySubsystem() {
        // Connect to active OpMode's telemetry instance
        RCTelemetry = RobotContainer.ActiveOpMode.telemetry;
    }

    /**
     * Method called periodically by the scheduler.
     *
     * @author Kw126
     */
    @Override
    public void periodic() {
        // Periodic telemetry updates if required
    }

    /**
     * Placeholder method for custom testing telemetry.
     *
     * @author Kw126
     */
    public void testingTelemetryEmpty() {
        if (testingTelemetry) {
            // Telemetry goes here
        }
    }

    /**
     * Placeholder method for custom operator telemetry.
     *
     * @author Kw126
     */
    public void operatorTelemetryEmpty() {
        if (!testingTelemetry) {
            // Telemetry goes here
        }
    }

    /**
     * Adds an item to the end of the telemetry being built for driver station display in testing mode.
     *
     * @author Kw126
     *
     * @param caption the caption to use
     * @param value the value to display
     */
    public void addData(String caption, Object value) {
        // Only output if currently in testing telemetry mode
        if (testingTelemetry) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     * Adds an item to the telemetry being built for driver station display, routed by mode flag.
     *
     * @author Kw126
     *
     * @param caption the caption to use
     * @param value the value to display
     * @param telemetryForTesting if true the data will be shown with testing data; if false, with operator data
     */
    public void addData(String caption, Object value, boolean telemetryForTesting) {
        // Route to telemetry output when the active mode matches telemetryForTesting
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     * Creates and outputs a new line in the receiver Telemetry in testing mode.
     *
     * @author Kw126
     *
     * @param caption the caption for the line
     */
    public void addLine(String caption) {
        // Add line if in testing telemetry mode
        if (testingTelemetry) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     * Creates and outputs a new line in the receiver Telemetry, routed by mode flag.
     *
     * @author Kw126
     *
     * @param caption the caption for the line
     * @param telemetryForTesting if true the data will be shown with testing data; if false, with operator data
     */
    public void addLine(String caption, boolean telemetryForTesting) {
        // Output line when current mode matches telemetryForTesting
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     * Displays robot fieldX, fieldY, and yaw heading from odometry.
     * Runs when in testing telemetry mode.
     *
     * @author Kw126
     */
    public void odometryTelemetry() {
        // Display position and heading if testing mode is active
        if (testingTelemetry) {
            Pose2d position = RobotContainer.odometry.getCurrentPos();
            RCTelemetry.addData("fieldX", position.getX());
            RCTelemetry.addData("fieldY", position.getY());
            RCTelemetry.addData("Yaw", position.getRotation().getDegrees());
        }
    }

    /**
     * Displays loop interval time and execution time.
     * Runs when in testing telemetry mode.
     *
     * @author Kw126
     */
    public void timerOdometry() {
        // Output timing metrics
        if (testingTelemetry) {
            RCTelemetry.addData("interval time(ms)", RobotContainer.intervaltime);
            RCTelemetry.addData("execute time(ms)", RobotContainer.exectimer.milliseconds());
        }
    }

    /**
     * Displays ASCII art of the number 0.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_0() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 1.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_1() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■□□");
            RCTelemetry.addLine("◢■■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("■■■■■■■");
            RCTelemetry.addLine("■■■■■■■");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 2.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_2() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■◤□□◥■■");
            RCTelemetry.addLine("□□□□□◢■◤");
            RCTelemetry.addLine("□□□□◢■◤□");
            RCTelemetry.addLine("□□□◢■◤□□");
            RCTelemetry.addLine("□□◢■◤□□□");
            RCTelemetry.addLine("◢■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 3.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_3() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■◤□□◥■■");
            RCTelemetry.addLine("□□□□□◢■◤");
            RCTelemetry.addLine("□□□□■■■□");
            RCTelemetry.addLine("□□□□□◥■◣");
            RCTelemetry.addLine("■■◣□□◢■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 4.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_4() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■■");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 5.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_5() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("■■■□□□□□");
            RCTelemetry.addLine("■■■■■■◣□");
            RCTelemetry.addLine("■■■■■■■◣");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("■■■■■■■◤");
            RCTelemetry.addLine("■■■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 6.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_6() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■■■");
            RCTelemetry.addLine("◢■■■■■■■");
            RCTelemetry.addLine("■■■□□□□□");
            RCTelemetry.addLine("■■■■■■◣□");
            RCTelemetry.addLine("■■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 7.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_7() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□◢■■◤");
            RCTelemetry.addLine("□□□◢■■◤□");
            RCTelemetry.addLine("□□◢■■◤□□");
            RCTelemetry.addLine("□◢■■◤□□□");
            RCTelemetry.addLine("◢■■◤□□□□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 8.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_8() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Displays ASCII art of the number 9.
     * Runs when in operator telemetry mode.
     *
     * @author Kw126
     */
    public void ascii_9() {
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■■");
            RCTelemetry.addLine("□◥■■■■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("");
        }
    }

    /**
     * Sends the buffered telemetry data to the FTC driver station display.
     *
     * @author Kw126
     */
    public void update() {
        // Transmit telemetry buffer to driver station
        RCTelemetry.update();
    }

    /**
     * Toggles the telemetry mode between testing (detailed) and operator (minimal/display).
     *
     * @author Kw126
     */
    public void telemetryModeToggle() {
        // Toggle the telemetry mode flag
        testingTelemetry = !testingTelemetry;
    }
}