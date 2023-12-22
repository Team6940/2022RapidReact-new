// package frc.robot.Auto;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.ShootBall;

// public class OneBallAutoCommand extends SequentialCommandGroup{
//     public OneBallAutoCommand()
//     {
//         PathPlannerTrajectory mOneBallTrajectoryOne = PathPlanner.loadPath("OneBallAuto", AutoConstants.kMaxSpeedMetersPerSecond-4, AutoConstants.kMaxAccelerationMetersPerSecondSquared-2);
//         addCommands(
//             // new InstantCommand(()->RobotContainer.m_Intake.SetIntakeState(0.6, true)),
//             FollowPathCommandGenerator.followTrajectoryCommand(mOneBallTrajectoryOne, true),
//             new ShootBall(true).raceWith(new WaitCommand(5))
//             );
//     }
// }