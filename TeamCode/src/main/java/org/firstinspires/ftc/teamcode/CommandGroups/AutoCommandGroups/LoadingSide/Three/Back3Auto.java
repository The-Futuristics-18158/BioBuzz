package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups.LoadingSide.Three;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.TeleOpSequences.ShotSequence;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

public class Back3Auto extends SequentialCommandGroup {

    // Constructor
    public Back3Auto() {

        addCommands (
                // Was X = 1.59 Y Was -0.39
                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(1.60, -0.37, new Rotation2d(Math.toRadians(180.0)))))),


                // Move to shoot
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.34, -0.38, new Rotation2d(Math.toRadians(-157.0)))))), // + or - 20 degrees

                // Waiting for spin up (we have the time)
                new Pause(1.5),

//      -------------------------- Artifact Cycle #1 --------------------------
                new ShotSequence(),

                // Leave
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -0.9, new Rotation2d(Math.toRadians(180.0))))))
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