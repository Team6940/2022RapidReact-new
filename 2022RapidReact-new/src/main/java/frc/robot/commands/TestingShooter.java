// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HopperConstants;
import frc.robot.lib.team3476.net.editing.LiveEditableValue;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestingShooter extends CommandBase {
  

  public TestingShooter() {
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Hood);
  }
  LiveEditableValue<Double> ShooterRPMTable = new LiveEditableValue<>(0.,
  SmartDashboard.getEntry("ShooterRPM"));
  LiveEditableValue<Double> HoodAngleTable = new LiveEditableValue<>(0.,
  SmartDashboard.getEntry("HoodAngle"));
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driverController.getBButtonPressed())
      RobotContainer.m_Shooter.SetSpeed(ShooterRPMTable.get());
    if(RobotContainer.m_driverController.getYButtonPressed())
    {
      RobotContainer.m_Hood.SetHoodAngle(HoodAngleTable.get());
    }
    // if(RobotContainer.m_driverController.getBButtonReleased());
    //   RobotContainer.m_Shooter.SetSpeed(0);
    if(RobotContainer.m_driverController.getRightBumper())
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(1.);
      // RobotContainer.m_Hopper.SetUpBackOutput(1.);
    }
    else
    {
      RobotContainer.m_Shooter.SetBlockerOutPut(0.);
      // RobotContainer.m_Hopper.SetUpBackOutput(0.);

    }
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
