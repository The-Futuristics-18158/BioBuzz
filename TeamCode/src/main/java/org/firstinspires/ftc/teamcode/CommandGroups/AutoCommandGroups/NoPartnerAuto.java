package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.IntakeSequence;
import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeRunEndless;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

/**
 *
 * @author Colm
 */
public class NoPartnerAuto extends SequentialCommandGroup {

    /**
     *
     * @author Colm
     */
    public NoPartnerAuto() {
        addCommands(
                // Add individual commands or nested command groups here
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 0.3, new Rotation2d(Math.toRadians(0.0)))),
                new ShotSequence(),
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 0.9, new Rotation2d(Math.toRadians(90.0)))),
                // Jeff - modified this for demo of how the intake runs while moving.  Intake stops at end of IntakeRunEndless in .end()
                new ParallelRaceGroup(
                        new IntakeRunEndless(),
                        new MoveToPose(1.5,1.0, AutoFunctions.redVsBlue((new Pose2d(-1.56, 1.5, new Rotation2d(Math.toRadians(90))))))),
                // Jeff - on Sunday these commands need to be moved into the SoloGarden CommandGRoup.
                new MoveToPose(1.0, 1.0, new Pose2d(0.9, 0.9, new Rotation2d(Math.toRadians(180.0)))),
                new MoveToPose(1.0, 1.0, new Pose2d(1.56, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                new ShotSequence(),
                new MoveToPose(1.0, 1.0, new Pose2d(1.5, 0.35, new Rotation2d(Math.toRadians(20.0)))),
                new IntakeSequence(),
                new MoveToPose(1.0, 1.0, new Pose2d(1.56, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                new ShotSequence(),
                new MoveToPose(1.0, 1.0, new Pose2d(1.2, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                new MoveToPose(1.0, 1.0, new Pose2d(1.2, 1.4, new Rotation2d(Math.toRadians(180.0))))







                );
    }
}