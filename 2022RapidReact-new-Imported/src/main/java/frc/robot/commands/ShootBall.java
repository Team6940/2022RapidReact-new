// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootBall extends Command {
  
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
  /**
   * 
   */
  void Aim()
  {
    RobotContainer.m_Limelight.setLightMode(3);//把limelight的的灯打开
    double _tv=RobotContainer.m_Limelight.Get_tv();//获取limelight是否看到视觉目标
    RobotContainer.m_Shooter.SetBlockerOutPut(0.);//默认将blocker的转速设置为0
    // SmartDashboard.putNumber("Distance" ,);
    if(_tv==1.)//如果你看到视觉目标
      {
        double _tx=RobotContainer.m_Limelight.Get_tx();//获取视觉目标的x坐标
        if(_tx<-ShooterConstants.ShootingDegreeTolerance)//如果视觉目标的x坐标偏左
          RobotContainer.m_swerve.Drive(new Translation2d(), ShooterConstants.ShootingFixSpeed, false, false);//向左转
        else if(_tx>ShooterConstants.ShootingDegreeTolerance)//如果视觉目标偏右
          RobotContainer.m_swerve.Drive(new Translation2d(), -ShooterConstants.ShootingFixSpeed, false, false);//向右转
        else//如果视觉目标的位置刚刚好
        {
                
        RobotContainer.m_swerve.Drive(new Translation2d(), 0, false, false);//让我的机器停止旋转
          m_TargetDistance=RobotContainer.m_Limelight.getRobotToTargetDistance()-0.23;//获取我和视觉目标的距离
          m_ShootRPM=ShooterConstants.kRPMTable.getOutput(m_TargetDistance);//获取shoooter的转速
          m_ShootHood=ShooterConstants.kHoodTable.getOutput(m_TargetDistance);//获取shooter的背轮角度
          RobotContainer.m_Limelight.setLightMode(1);//把limelight关掉
          m_State=ShooterState.Accelerating;//切近加速状态
      
        }
      }
      else{
        RobotContainer.m_swerve.Drive(new Translation2d(), 0, false, false);
  
      }
  }
  void Accelerate()
  {
    RobotContainer.m_Shooter.SetSpeed(m_ShootRPM);//设置shooter转速为目标转速
    RobotContainer.m_Hood.SetHoodAngle(m_ShootHood);//设置shooter角度为目标出球角度
    if(RobotContainer.m_Shooter.IsSpeedReached()&&RobotContainer.m_Hood.IsHoodAtAngle())//如果shooter打到目标转速，hood达到目标出球角度
    {
      m_State=ShooterState.Shooting;//把状态切进射球状态
    }

  }
  void Shoot()
  {
    if(!RobotContainer.m_Hopper.NoBallOnTop())//如果球路上面有球
      RobotContainer.m_Shooter.SetBlockerOutPut(1.0);//blocker转动
    else//否则,如果没球
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(0.);//blocker不要转动
      m_State=ShooterState.Accelerating;//又切进加速状态
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    RobotContainer.m_Shooter.SetBlockerOutPut(0.);
    RobotContainer.m_Shooter.SetSpeed(600);
    RobotContainer.m_Limelight.setLightMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.m_driverController.getRightBumper()&&!m_IsAuto;
  }
}
