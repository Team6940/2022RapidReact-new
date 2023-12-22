// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoFeed;
import frc.robot.commands.HandShootBall;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TestingShooter;
import frc.robot.commands.SwerveControl.SemiAutoSwerveControll;
import frc.robot.commands.SwerveControl.SwerveControll;
import frc.robot.subsystems.Shooter;
import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import java.lang.Math;
;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  int m_BallAuto=1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
     Pathfinding.setPathfinder(new LocalADStar());

    // CameraServer.startAutomaticCapture();
    RobotContainer.m_swerve.setDefaultCommand(new SwerveControll());
    RobotContainer.m_Hopper.setDefaultCommand(new AutoFeed());
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(RobotContainer.m_driverController.getXButtonPressed())
    {
      m_BallAuto+=1;

    }
    if(RobotContainer.m_driverController.getYButtonPressed())
    {
      m_BallAuto-=1;
    }
    SmartDashboard.putNumber("Auto", m_BallAuto);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
      
    // RobotContainer.m_swerve.ZeroHeading();
    // if(m_BallAuto==5)
    //   new FiveBallAutoCommand().withTimeout(15.).schedule();
    // if(m_BallAuto==2)
    //   new TwoBallAutoCommand().raceWith(new WaitCommand(15)).schedule();
    // if(m_BallAuto==1)
    //   new OneBallAutoCommand().raceWith(new WaitCommand(15)).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.m_Limelight.setLightMode(1);
    // new TestingShooter().schedule();
    
    // RobotContainer.m_swerve.ZeroHeading();
    // RobotContainer.m_swerve.CollaborateGyro();
    RobotContainer.m_swerve.whetherstoreyaw = false;
    m_robotContainer = new RobotContainer();
    RobotContainer.m_swerve.ZeroHeading();;
  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(RobotContainer.m_driverController.getAButtonPressed())
    // {
    //   // new SemiAutoSwerveControll(new Pose2d(),1).schedule();;
      
    //   SemiAutoTrajCommandGenerator.SemiAutoPPSControl(new Pose2d[]{RobotContainer.m_swerve.getPose(), 
    //     new Pose2d(2,0,new Rotation2d(Math.PI)),
    //     new Pose2d(0,0,new Rotation2d(Math.PI))}).schedule();
    // }
    // if(RobotContainer.m_driverController.getBButtonPressed())
    // {
      
    //   SemiAutoTrajCommandGenerator.SemiAutoPPSControl(new Pose2d[]{RobotContainer.m_swerve.getPose(), 
    //     new Pose2d(2,0,new Rotation2d(Math.PI/2.)),
    //     new Pose2d(2,-2,new Rotation2d(Math.PI/2.))}).schedule();;
    // }
    // if(RobotContainer.m_driverController.getXButtonPressed())
    // {
    //     SemiAutoTrajCommandGenerator.SemiAutoPPSControl(new Pose2d[]{RobotContainer.m_swerve.getPose(), 
    //     new Pose2d(2,0,new Rotation2d(-Math.PI/2.)),
    //     new Pose2d(2,2,new Rotation2d(-Math.PI/2.))}).schedule();;}
    // if(RobotContainer.m_driverController.getYButtonPressed())
    // { SemiAutoTrajCommandGenerator.SemiAutoPPSControl(new Pose2d[]{RobotContainer.m_swerve.getPose(), 
    //   new Pose2d(2,0,new Rotation2d(0)),
    //   new Pose2d(4,0,new Rotation2d(0))}).schedule();;
    // }
    
    
    if(RobotContainer.m_driverController.getRightBumperPressed())
    {
      Pose2d StartPose=PathPlannerPath.fromPathFile("Example Path").getPreviewStartingHolonomicPose();
      RobotContainer.m_swerve.ResetOdometry(StartPose);
    RobotContainer.m_swerve.followPathCommand("Example Path").schedule();
      
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
    if(RobotContainer.m_driverController.getPOV()==180||RobotContainer.m_testController.getPOV()==180)
    {
      RobotContainer.m_Climber.SetClimberOutput(1);
      
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
    else if(RobotContainer.m_driverController.getPOV()==0||RobotContainer.m_testController.getPOV()==0)
    {
      RobotContainer.m_Climber.SetClimberOutput(-1);;
      
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
    else
    {
      RobotContainer.m_Climber.SetClimberOutput(0);
    }
    if(RobotContainer.m_driverController.getYButtonPressed())
    {
      new HandShootBall().schedule();
    }
    if(RobotContainer.m_driverController.getAButton()||RobotContainer.m_testController.getAButton())
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(0.6);
    }
    else
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(0.);
    }

    if(RobotContainer.m_driverController.getLeftBumper())
    {
      RobotContainer.m_Intake.SetIntakeState(0.6, true);
    }
    else if(RobotContainer.m_driverController.getLeftTriggerAxis()>0.3)
    {
      RobotContainer.m_Intake.SetIntakeState(-0.4, false);
    }
    else 
    {
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
  }

  @Override
  public void testInit() {
    RobotContainer.m_Limelight.setLightMode(3);
    
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(RobotContainer.m_driverController.getPOV()==180||RobotContainer.m_testController.getPOV()==180)
    {
      RobotContainer.m_Climber.SetClimberOutput(1);
      
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
    else if(RobotContainer.m_driverController.getPOV()==0||RobotContainer.m_testController.getPOV()==0)
    {
      RobotContainer.m_Climber.SetClimberOutput(-1);;
      
      RobotContainer.m_Intake.SetIntakeState(0, false);
    }
    else
    {
      RobotContainer.m_Climber.SetClimberOutput(0);
    }
    if(RobotContainer.m_driverController.getYButtonPressed())
    {
      new HandShootBall().schedule();
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
