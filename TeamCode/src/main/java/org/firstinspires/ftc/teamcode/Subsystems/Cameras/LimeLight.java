package org.firstinspires.ftc.teamcode.Subsystems.Cameras;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Limelight 3A vision subsystem responsible for AprilTag fiducial detection, targeting,
 * and Megatag2 robot field localization.
 *
 * @author gbuckholtz
 */
public class LimeLight extends SubsystemBase {

    // Limelight hardware device interface
    private Limelight3A limeLight;

    /**
     * Initializes the Limelight camera from the hardware map, configures polling rate,
     * starts streaming, and sets pipeline mode to 0.
     *
     * @author gbuckholtz
     */
    public LimeLight() {
        // Retrieve the Limelight hardware device from the active OpMode's hardware map
        limeLight = RobotContainer.ActiveOpMode.hardwareMap.get(Limelight3A.class, "limeLight");

        // Set how frequently to query Limelight for updated vision data (50 Hz)
        limeLight.setPollRateHz(50);

        // Start vision capture pipeline on the Limelight
        limeLight.start();

        // Switch to default pipeline 0
        limeLight.pipelineSwitch(0);
    }

    /**
     * Method called periodically by the scheduler.
     *
     * @author gbuckholtz
     */
    @Override
    public void periodic() {
        // Future orientation updates can be passed here to aid MegaTag localization
    }

    /**
     * Switches the vision pipeline on the Limelight camera.
     *
     * @author gbuckholtz
     *
     * @param pipelineMode an integer representing the desired pipeline mode: 0 for obelisk/goal id detection, 1 for position tag detection, 2 for driver camera mode
     */
    public void SetPipelineMode(int pipelineMode) {
        // Verify requested pipeline mode is valid before switching
        if (pipelineMode == 0 || pipelineMode == 1 || pipelineMode == 2) {
            limeLight.pipelineSwitch(pipelineMode);
        }
    }

    /**
     * Retrieves the latest vision detection results from the Limelight camera.
     *
     * @author gbuckholtz
     *
     * @return the latest LLResult containing fiducial results, botpose, and targeting metrics
     */
    public LLResult getLimeLightResults() {
        // Query the latest result from the Limelight driver
        return limeLight.getLatestResult();
    }
}