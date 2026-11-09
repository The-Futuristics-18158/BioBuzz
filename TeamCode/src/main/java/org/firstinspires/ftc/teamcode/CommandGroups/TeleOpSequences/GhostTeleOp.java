package org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.IntakeRunEndless;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

/**
 * Automated sequential routine for TeleOp ("Ghost TeleOp").
 * Automatically navigates to an intake pose while intaking, navigates to shooting position,
 * and executes a shot sequence.
 *
 * @author gbuckholtz
 */
public class GhostTeleOp extends SequentialCommandGroup {

    /**
     * Constructs a GhostTeleOp sequential command group.
     * Coordinates driving, intaking, and shooting steps.
     *
     * @author gbuckholtz
     */
    public GhostTeleOp() {
        addCommands(
                // Move to artifact pickup location while running intake
                new ParallelRaceGroup(
                        new MoveToPose(
                                1.5,
                                1.0,
                                AutoFunctions.redVsBlue(new Pose2d(1.34, 1.4, new Rotation2d(Math.toRadians(90.0))))),
                        new IntakeRunEndless()
                ),

                // Move to shooting location
                new MoveToPose(
                        1.5,
                        1.5,
                        AutoFunctions.redVsBlue(new Pose2d(1.34, 0.3, new Rotation2d(Math.toRadians(90.0))))),

                // Execute shooting sequence
                new ShotSequence()
        );
    }

    /**
     * Called when the routine ends or is interrupted.
     *
     * @author gbuckholtz
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
    }
}