package frc.robot.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDriveTrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import edu.wpi.first.wpilibj2.command.Command;

public class FollowPathCommandGenerator {
    public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   RobotContainer.m_swerve.ResetOdometry(traj.getInitialHolonomicPose());
               }
             }),new PPSwerveControllerCommand(
                traj,
                RobotContainer.m_swerve::getPose, // Functional interface to feed supplier
                SwerveDriveTrain.kDriveKinematics,
          
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController),
                RobotContainer.m_swerve::SetModuleStates,
                RobotContainer.m_swerve)
             )
         ;
     }
}
