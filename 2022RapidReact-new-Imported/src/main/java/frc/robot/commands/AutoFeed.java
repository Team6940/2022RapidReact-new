// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoFeed extends Command {
  
  
  public AutoFeed() {
    addRequirements(RobotContainer.m_Hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driverController.getBButton()||RobotContainer.m_testController.getBButton())
{
  RobotContainer.m_Hopper.SetBottomOutput(HopperConstants.HopperOutput);
      RobotContainer.m_Hopper.SetUpFrontOutput(HopperConstants.HopperOutput);
      RobotContainer.m_Hopper.SetUpBackOutput(HopperConstants.HopperOutput);
}
    else if(!RobotContainer.m_Hopper.NoBallOnBottom()&&!RobotContainer.m_Hopper.NoBallOnTop())
    {
      RobotContainer.m_Hopper.SetBottomOutput(0);
      RobotContainer.m_Hopper.SetUpFrontOutput(0);
      RobotContainer.m_Hopper.SetUpBackOutput(0);
    }
    else if(RobotContainer.m_Hopper.NoBallOnBottom()&&!RobotContainer.m_Hopper.NoBallOnTop())
      {
        RobotContainer.m_Hopper.SetBottomOutput(HopperConstants.HopperOutput);
        RobotContainer.m_Hopper.SetUpBackOutput(0);
        RobotContainer.m_Hopper.SetUpFrontOutput(0);
      
      
      }
    else if(!RobotContainer.m_Hopper.NoBallOnBottom()&&RobotContainer.m_Hopper.NoBallOnTop())
    {
      RobotContainer.m_Hopper.SetBottomOutput(HopperConstants.HopperOutput);
      RobotContainer.m_Hopper.SetUpFrontOutput(HopperConstants.HopperOutput);
      RobotContainer.m_Hopper.SetUpBackOutput(HopperConstants.HopperOutput);
    }
    else
    {
      RobotContainer.m_Hopper.SetBottomOutput(HopperConstants.HopperOutput);
      RobotContainer.m_Hopper.SetUpFrontOutput(0);
      RobotContainer.m_Hopper.SetUpBackOutput(0);

    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
    RobotContainer.m_Hopper.SetBottomOutput(0);
    RobotContainer.m_Hopper.SetUpBackOutput(0);
    RobotContainer.m_Hopper.SetUpFrontOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
