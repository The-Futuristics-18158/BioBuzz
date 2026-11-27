package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;

/**
 *
 * @author Colm
 */
public class LeftAuto extends SequentialCommandGroup {

    /**
     *
     * @author Colm
     */
    public LeftAuto() {
        addCommands(
                // Add individual commands or nested command groups here
                new ShotSequence(),
                new Pause(2.0),
                new MoveToPose(1.0, 1.0, new Pose2d(1.5, 0.35, new Rotation2d(Math.toRadians(20.0)))),
                new IntakeCommand(),
                new MoveToPose(1.0, 1.0, new Pose2d(1.56, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                new ShotSequence(),
                new Pause(2.0),
                new MoveToPose(1.0, 1.0, new Pose2d(1.2, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                new MoveToPose(1.0, 1.0, new Pose2d(1.2, 1.4, new Rotation2d(Math.toRadians(180.0))))







                );
    }
}