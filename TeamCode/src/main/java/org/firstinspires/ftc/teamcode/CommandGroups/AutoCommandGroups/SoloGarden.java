package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.IntakeRunEndless;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

public class SoloGarden extends SequentialCommandGroup {

    // Constructor
    public SoloGarden() {

        addCommands (
                // Was X = 1.59 Y Was -0.39
                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(-1.6, 0.3, new Rotation2d(Math.toRadians(0.0)))))),

                //shot sequence
                new ShotSequence(),
                new Pause(2.0),
                new ShotSequence(),
                new Pause(2.0),
                new ShotSequence(),
                new Pause(2.0),
                new ShotSequence(),
                new Pause(2.0),

                // Move to pick up point beside the from garden
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(-1.56, 0.9, new Rotation2d(Math.toRadians(90)))))),

               // Turn on Intake and Move to Corner of Garden
                new ParallelRaceGroup(
                        new IntakeRunEndless(),
                        new MoveToPose(
                                1.5,
                                1.0,
                                AutoFunctions.redVsBlue((new Pose2d(-1.56, 1.5, new Rotation2d(Math.toRadians(90)))))))

               //
        );
    }
}

//Example MoveToPose
// new MoveToPose(
//           1.5,
//           1.0,
// AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.2, new Rotation2d(Math.toRadians(-90))))
//        ),

//Example FollowPath
// new FollowPath(
//             2.0,
//             1.0,
//             0.0,
//             0.0,
// AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
// new ArrayList<Translation2d>() {{ }},
// AutoFunctions.redVsBlue(new Pose2d(0.55, 0.25, new Rotation2d(Math.toRadians(-180)))),
// AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-180)))),


// Example #2: Using conditional command
// conditionally run commands depending on condition
/*      new FourWayConditionalCommand(
            ()-> { return true; },
            new SequentialCommandGroup(
                commands to do this
            ),
            ()-> { return false; },
            new SequentialCommandGroup(
                commands to do that
            ),
            ()-> { return false; },
            new SequentialCommandGroup(
                commands to do those
            ),
            new SequentialCommandGroup(
                commands to do something else
            )

        ) // end FourWayCondition
*/