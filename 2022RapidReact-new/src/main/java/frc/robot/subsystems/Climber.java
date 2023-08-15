// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  static Climber m_Instance;
  WPI_TalonFX m_ClimberMotor;
  /** Creates a new ExampleSubsystem. */
  public Climber()
  {
    ClimberConfig();
  }
  void ClimberConfig()
{
  m_ClimberMotor=new WPI_TalonFX(ClimberConstants.ClimberPort);
  m_ClimberMotor.setInverted(true);
  m_ClimberMotor.config_kP(0, 0.15);
  m_ClimberMotor.config_kI(0, 0);
  m_ClimberMotor.config_kD(0, 0);
  m_ClimberMotor.setNeutralMode(NeutralMode.Brake);
  
  m_ClimberMotor.config_IntegralZone(0, 0, 10);
  m_ClimberMotor.configMotionCruiseVelocity(600, 10);//???
  m_ClimberMotor.configMotionAcceleration(1200, 10);//???
}
  public static Climber GetInstance()
  {
    return m_Instance==null? m_Instance=new Climber():m_Instance;
  }
  public void SetClimberOutput(double _PCT)
  {
    m_ClimberMotor.set(ControlMode.PercentOutput,_PCT);
  }
  public void SetClimberOut()
  {
    m_ClimberMotor.set(ControlMode.MotionMagic, ClimberConstants.ClimberOutUnits);
  }
  public void SetClimberBack()
  {
    m_ClimberMotor.set(ControlMode.MotionMagic, ClimberConstants.ClimberBackUnits);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
