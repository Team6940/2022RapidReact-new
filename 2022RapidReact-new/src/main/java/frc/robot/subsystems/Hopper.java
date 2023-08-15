// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static Hopper m_Instance;
  public static Hopper GetInstance()
  {
    return m_Instance==null? m_Instance=new Hopper():m_Instance;
  }
  public Hopper() {
    ConfigHopper();
  }
  WPI_TalonFX m_UpFrontMotor;
  WPI_TalonFX m_UpBackMotor;
  WPI_TalonSRX m_BottomMotor;

  DigitalInput m_UpBallSensor;
  DigitalInput m_DownBallSensor;
  DigitalInput m_DownFrontBallSensor;

  
  private void ConfigHopper()
  {
    m_UpFrontMotor=new WPI_TalonFX(HopperConstants.UpFrontMotorPort);
    m_UpBackMotor=new WPI_TalonFX(HopperConstants.UpBackMotorPort);
    m_BottomMotor=new WPI_TalonSRX(HopperConstants.BottomMotorPort);
    m_UpBallSensor=new DigitalInput(HopperConstants.UpBallSensorChannel);
    m_DownBallSensor=new DigitalInput(HopperConstants.DownBallSensor);

    m_UpFrontMotor.setNeutralMode(NeutralMode.Brake);
    m_UpBackMotor.setNeutralMode(NeutralMode.Brake);
    m_BottomMotor.setNeutralMode(NeutralMode.Brake);
    
    m_UpFrontMotor.setInverted(true);
    m_UpBackMotor.setInverted(true);
    m_BottomMotor.setInverted(true);

    m_UpFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_UpBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_BottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    
  }
  public void SetUpFrontOutput(double _UpFrontOutput)
  {
    m_UpFrontMotor.set(ControlMode.PercentOutput,_UpFrontOutput);
  }
  public void SetUpBackOutput(double _UpBackOutput)
  {
    m_UpBackMotor.set(ControlMode.PercentOutput,_UpBackOutput);
  }
  public void SetBottomOutput(double _BottomOutput)
  {
    m_BottomMotor.set(ControlMode.PercentOutput,_BottomOutput);
  }
  public boolean NoBallOnTop()
  {
    return m_UpBallSensor.get();
  }
  public boolean NoBallOnBottom()
  {
    return m_DownBallSensor.get();
  }
  public boolean IsBallOnFront()
  {
    return m_DownFrontBallSensor.get();
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
