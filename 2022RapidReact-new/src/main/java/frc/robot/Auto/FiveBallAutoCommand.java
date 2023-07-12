package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;

public class FiveBallAutoCommand extends SequentialCommandGroup{
    public FiveBallAutoCommand()
    {
        PathPlannerTrajectory mFiveBallTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", AutoConstants.kFastMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", AutoConstants.kFastMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryThree = PathPlanner.loadPath("FiveBallBottom-3", AutoConstants.kFastMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryFour = PathPlanner.loadPath("FiveBallBottom-4", AutoConstants.kFastMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerTrajectory mFiveBallTrajectoryFive = PathPlanner.loadPath("FiveBallBottom-5", AutoConstants.kFastMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        addCommands(FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryOne, true)
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryTwo, false)
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryThree, false)
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFour, false)
            ,       FollowPathCommandGenerator.followTrajectoryCommand(mFiveBallTrajectoryFive, false));
    }
}