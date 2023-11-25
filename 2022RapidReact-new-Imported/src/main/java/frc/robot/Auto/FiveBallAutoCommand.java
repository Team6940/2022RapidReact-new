package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ShootBall;

public class FiveBallAutoCommand extends SequentialCommandGroup{
    public FiveBallAutoCommand()
    {
        PathPlannerTrajectory mFiveBallTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared+1);
        PathPlannerTrajectory mFiveBallTrajectoryThree = PathPlanner.loadPath("FiveBallBottom-3", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared+1);
        PathPlannerTrajectory mFiveBallTrajectoryFour = PathPlanner.loadPath("FiveBallBottom-4", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryFive = PathPlanner.loadPath("FiveBallBottom-5", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared+1);
        addCommands(
            new InstantCommand(()->RobotContainer.m_Intake.SetIntakeState(0.6, true)),
            FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryOne, true),
                  FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryTwo, false),
                  new ShootBall(true).raceWith(new WaitCommand(3))
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryThree, false)
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFour, false),
            // ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFive, false),
            new ShootBall(true)
            );
        // PathPlannerTrajectory mFiveBallTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        // PathPlannerTrajectory mFiveBallTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared+1);
        // PathPlannerTrajectory mFiveBallTrajectoryFour = PathPlanner.loadPath("FiveBallBottom-4", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        // PathPlannerTrajectory mFiveBallTrajectoryFive = PathPlanner.loadPath("FiveBallBottom-5", AutoConstants.kMaxSpeedMetersPerSecond+2, AutoConstants.kMaxAccelerationMetersPerSecondSquared+1);
        // addCommands(
        //     new InstantCommand(()->RobotContainer.m_Intake.SetIntakeState(0.6, true)),
        //     FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryOne, true),
        //           FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryTwo, false),
                  
        //     new ShootBall(true).raceWith(new WaitCommand(3))
        //     ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFour, false),
            
        //            FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFive, false),
        //     new ShootBall(true)
        //     );
    }
}