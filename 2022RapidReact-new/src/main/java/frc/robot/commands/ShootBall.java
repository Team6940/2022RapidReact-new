// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShootBall extends CommandBase {
  
  public enum ShooterState {
    Aiming,Accelerating,Shooting
  }
  ShooterState m_State;
  double m_TargetDistance;
  double m_ShootRPM;
  double m_ShootHood;
  boolean m_IsAuto;
  public ShootBall(boolean _IsAuto) {
    addRequirements(RobotContainer.m_swerve);
    addRequirements(RobotContainer.m_Hood);
    
  addRequirements(RobotContainer.m_Shooter);
m_IsAuto=_IsAuto;
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_State=ShooterState.Aiming;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      if(m_State==ShooterState.Aiming)
      Aim();
      if(m_State==ShooterState.Accelerating)
      Accelerate();
      if(m_State==ShooterState.Shooting)
      Shoot();

   
    SmartDashboard.putString("shooterState", m_State.toString());
  }
  void ChangeStateTo(ShooterState _State)
  {
  }
  void Aim()
  {
    RobotContainer.m_Limelight.setLightMode(3);
    double _tv=RobotContainer.m_Limelight.Get_tv();
    RobotContainer.m_Shooter.SetBlockerOutPut(0.);
    // SmartDashboard.putNumber("Distance" ,);
    if(_tv==1.)
      {
        double _tx=RobotContainer.m_Limelight.Get_tx();
        if(_tx<-ShooterConstants.ShootingDegreeTolerance)
          RobotContainer.m_swerve.Drive(new Translation2d(), ShooterConstants.ShootingFixSpeed, false, false);
        else if(_tx>ShooterConstants.ShootingDegreeTolerance)
          RobotContainer.m_swerve.Drive(new Translation2d(), -ShooterConstants.ShootingFixSpeed, false, false);
        else
        {
                
        RobotContainer.m_swerve.Drive(new Translation2d(), 0, false, false);
          m_TargetDistance=RobotContainer.m_Limelight.getRobotToTargetDistance()-0.3;
          m_ShootRPM=ShooterConstants.kRPMTable.getOutput(m_TargetDistance);
          m_ShootHood=ShooterConstants.kHoodTable.getOutput(m_TargetDistance);
          RobotContainer.m_Limelight.setLightMode(1);
          m_State=ShooterState.Accelerating;
      
        }
      }
      else{
        RobotContainer.m_swerve.Drive(new Translation2d(), 0, false, false);
  
      }
  }
  void Accelerate()
  {
    RobotContainer.m_Shooter.SetSpeed(m_ShootRPM);
    RobotContainer.m_Hood.SetHoodAngle(m_ShootHood);
    if(RobotContainer.m_Shooter.IsSpeedReached()&&RobotContainer.m_Hood.IsHoodAtAngle())
    {
      m_State=ShooterState.Shooting;
    }

  }
  void Shoot()
  {
    if(!RobotContainer.m_Hopper.NoBallOnTop())
      RobotContainer.m_Shooter.SetBlockerOutPut(1.0);
    else
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(0.);
      m_State=ShooterState.Accelerating;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    RobotContainer.m_Shooter.SetBlockerOutPut(0.);
    RobotContainer.m_Shooter.SetSpeed(0);
    RobotContainer.m_Limelight.setLightMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.m_driverController.getRightBumper()&&!m_IsAuto;
  }
}
