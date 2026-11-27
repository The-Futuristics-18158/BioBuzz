package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 *
 * @author Colm
 */
public class RightAuto extends SequentialCommandGroup {

    /**
     *
     * @author Colm
     */
    public RightAuto() {
        addCommands(
                // Add individual commands or nested command groups here
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 0.3, new Rotation2d(Math.toRadians(0.0)))),
                new ShotSequence(),
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 0.9, new Rotation2d(Math.toRadians(90.0)))),
                new InstantCommand(() -> RobotContainer.intake.intakeRun()),
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 1.5, new Rotation2d(Math.toRadians(90.0)))),
                new InstantCommand(() -> RobotContainer.intake.intakeStop()),
                new MoveToPose(1.0, 1.0, new Pose2d(-1.56, 0.3, new Rotation2d(Math.toRadians(0.0)))),
               //delay as long as posible
                //shoot
                new MoveToPose(1.0, 1.0, new Pose2d(-0.6, 1.1, new Rotation2d(Math.toRadians(0.0)))),
                new MoveToPose(1.0, 1.0, new Pose2d(0.8, 1.4, new Rotation2d(Math.toRadians(0.0))))








                );
    }
}