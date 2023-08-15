// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.team1678.math.Conversions;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static Shooter m_Instance;
  private static WPI_TalonFX m_ShooterLeft;//左shooter电机
  private static WPI_TalonFX m_ShooterRght;//右shooter电机
  
  private static WPI_TalonFX m_Blocker;
  private double m_TargetSpeed;
  public boolean m_Enabled=true;
  public Shooter() {
    ShooterConfig();
  }
  public static Shooter GetInstance()
  {
    return m_Instance==null? m_Instance=new Shooter():m_Instance;
  }
  private void ShooterConfig()
  {
    //shooter电机参数设定
    m_Blocker = new WPI_TalonFX(ShooterConstants.BlockerMotorPort);
    m_Blocker.setNeutralMode(NeutralMode.Brake);
    m_Blocker.setInverted(true);
    m_ShooterLeft = new WPI_TalonFX(ShooterConstants.SHOOT_L_MASTER_ID);//设定shooter电机
    m_ShooterLeft.setInverted(true);//TODO 电机是否反转
    //mShooterLeft.configAllSettings(lMasterConfig);//将pid参数注入到电机中
    m_ShooterLeft.setNeutralMode(NeutralMode.Coast);
    m_ShooterLeft.config_kP(0, 0.3);
    m_ShooterLeft.config_kI(0, 0);
    m_ShooterLeft.config_kD(0, 0.13);
    m_ShooterLeft.config_kF(0, 0.07);
    m_ShooterLeft.configPeakOutputForward(1);
    m_ShooterLeft.configPeakOutputReverse(-1);
    m_ShooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_ShooterLeft.configVoltageCompSaturation(10);
    m_ShooterLeft.enableVoltageCompensation(true);
    m_ShooterLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    m_ShooterLeft.configVelocityMeasurementWindow(1);
    //右电机，同上
    m_ShooterRght = new WPI_TalonFX(ShooterConstants.SHOOT_R_MASTER_ID);
    m_ShooterRght.setInverted(false);//TODO
    m_ShooterRght.follow(m_ShooterLeft);
    //mShooterRght.configAllSettings(lMasterConfig);
    m_ShooterRght.setNeutralMode(NeutralMode.Coast);
    m_ShooterRght.config_kP(0, 0.3);//0.0000005
    m_ShooterRght.config_kI(0, 0);
    m_ShooterRght.config_kD(0, 0.13);
    m_ShooterRght.config_kF(0, 0.07);//0.05
    m_ShooterRght.configPeakOutputForward(1.0);
    m_ShooterRght.configPeakOutputReverse(-1.0);
    m_ShooterRght.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_ShooterRght.configVoltageCompSaturation(10);
    m_ShooterRght.enableVoltageCompensation(true);
    m_ShooterRght.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    m_ShooterRght.configVelocityMeasurementWindow(1);
  }
  public void SetBlockerOutPut(Double _PCT)
  {
    m_Blocker.set(ControlMode.PercentOutput, _PCT);
  }
  public void SetSpeed(double _RPM)
  {
    
  if(!m_Enabled) return;
    m_TargetSpeed=_RPM;
    if(_RPM==0)
    m_ShooterLeft.stopMotor();
    else
    m_ShooterLeft.set(ControlMode.Velocity, Conversions.RPMToFalcon(_RPM,ShooterConstants.kFlyWheelEncoderReductionRatio));
    
  }
  public Boolean IsSpeedReached()
  {
    return Math.abs(GetSpeed()-GetTargetSpeed())<ShooterConstants.kShooterTolerance;
  }
  /**
   * 
   * @return The Speed of Shooter, in RPM
   */
  public double GetSpeed()
  {
    return Conversions.falconToRPM(m_ShooterLeft.getSelectedSensorVelocity(), ShooterConstants.kFlyWheelEncoderReductionRatio);
  }
  public double GetTargetSpeed()
  {
    return m_TargetSpeed;
  }
  public boolean IsTargetSpeedReached()
  {
    return (Math.abs(GetSpeed()-GetTargetSpeed())<=ShooterConstants.kShooterTolerance);
  }
  private void  DashboardOutput()
  {
    SmartDashboard.putNumber("ShooterPresentSpeed", GetSpeed());
    SmartDashboard.putNumber("ShooterTargetSpeed", GetTargetSpeed() );
    SmartDashboard.putBoolean("IsAtSpeed",IsTargetSpeedReached());

  }
  
  @Override
  public void periodic() {
    DashboardOutput();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
