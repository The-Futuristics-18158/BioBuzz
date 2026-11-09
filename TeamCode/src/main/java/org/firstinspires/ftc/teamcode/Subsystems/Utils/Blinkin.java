package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem that manages the REV Blinkin LED Driver to provide visual robot status indication.
 * Displays alliance colors, targeting locks, shooting safety zones, and vacuum modes.
 *
 * @author gbuckholtz
 */
public class Blinkin extends SubsystemBase {
    private RevBlinkinLedDriver blinkin;

    /**
     * Initializes the Blinkin LED subsystem from the hardware map.
     *
     * @author gbuckholtz
     */
    public Blinkin() {
        // Retrieve the REV Blinkin LED driver hardware from the active OpMode's hardware map
        blinkin = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    /**
     * Method called periodically by the scheduler to update the LED pattern based on robot state.
     * Evaluates alliance color, vision targeting lock, shot safety zone, and vacuum state.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Show team alliance color as default base pattern
        if (RobotContainer.isRedAlliance()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // Check if Limelight has a valid target for shooting (AprilTag ID 24 for Red, 20 for Blue)
        LLResult results = RobotContainer.limeLight.getLimeLightResults();
        if (results != null && results.isValid() && results.getFiducialResults() != null &&
                !results.getFiducialResults().isEmpty() && results.getStaleness() < 100) {
            for (int i = 0; i < results.getFiducialResults().size(); ++i) {
                if ((RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 24) ||
                        (!RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 20)) {
                    // Set purple/violet pattern when target is locked
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
            }
        }

        // Check if robot is in the safe zone to shoot
        if (RobotContainer.targeting.IsRobotInAllowableShotZone()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        // Show fancy green pattern if currently in vacuum mode
        if (RobotContainer.isVacuuming) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
        }
    }
}