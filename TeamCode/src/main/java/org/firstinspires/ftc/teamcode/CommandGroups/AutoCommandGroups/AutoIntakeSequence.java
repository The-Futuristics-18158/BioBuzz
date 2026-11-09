package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode.HuntModeAuto;
import org.firstinspires.ftc.teamcode.Commands.UptakeRampControle;

/**
 * Autonomous intake command sequence that runs automated artifact hunting and ramp uptake control in parallel.
 *
 * @author gbuckholtz
 */
public class AutoIntakeSequence extends SequentialCommandGroup {

    private double default_timeout;

    /**
     * Constructs an AutoIntakeSequence with a default timeout of 1.75 seconds.
     *
     * @author gbuckholtz
     */
    public AutoIntakeSequence() {
        default_timeout = 1.75;
        // Run hunt mode and uptake ramp control in parallel
        addCommands(
                new ParallelCommandGroup(
                        new HuntModeAuto(default_timeout),
                        new UptakeRampControle(default_timeout)
                )
        );
    }

    /**
     * Constructs an AutoIntakeSequence with a custom specified timeout.
     *
     * @author gbuckholtz
     *
     * @param timeout duration in seconds for the intake sequence
     */
    public AutoIntakeSequence(double timeout) {
        // Run hunt mode and uptake ramp control in parallel with custom duration
        addCommands(
                new ParallelCommandGroup(
                        new HuntModeAuto(timeout),
                        new UptakeRampControle(timeout)
                )
        );
    }
}