// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public boolean m_Enabled=true; 

  WPI_TalonFX m_IntakeMotor;
  Solenoid m_IntakeSolenoid;
  double m_MotorOutput;
  public static Intake m_Instance;
  public static Intake GetInstance()
  {
    return m_Instance==null? m_Instance=new Intake():m_Instance;
  }
  public Intake() {
    ConfigIntakeMotor();
  }
  void SetIntakeState(double MotorPct,boolean SolenoidState)
  {
    
  if(!m_Enabled) return;
    m_IntakeSolenoid.set(true);
    m_IntakeMotor.set(ControlMode.Velocity, MotorPct);
  }
  double GetMotorOutput()
  {
    return m_MotorOutput;
  }
  boolean GetSolenoidState()
  {
    return m_IntakeSolenoid.get();
  }
  void ConfigIntakeMotor()
  { 
    m_IntakeMotor = new WPI_TalonFX(IntakeConstants.IntakerPort); //TODO 设定intake电机
    m_IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.IntakerSolenoidPort); //TODO 设定气动杆
    m_IntakeMotor.setInverted(false);
    m_IntakeMotor.setNeutralMode(NeutralMode.Brake);
  }
  void DashboardOutput()
  {
    SmartDashboard.putNumber("IntakeMotorOutput", GetMotorOutput());
    SmartDashboard.putBoolean("IntakeSolenoidState", GetSolenoidState());
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
