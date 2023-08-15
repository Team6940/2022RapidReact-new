// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.lib.team1678.math.Conversions;

public class Hood extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private WPI_TalonFX m_Hoodmotor;//hood电机
  public boolean m_Enabled=true;
  private double m_TargetHoodAngle=0;
  public static Hood m_Instance;
  public static Hood GetInstance()
  {
    return m_Instance==null? m_Instance=new Hood():m_Instance;
  }
  public Hood() {
    HoodConfig();
  }
  private void HoodConfig(){//设定hood参数
    //hood电机参数
    m_Hoodmotor = new WPI_TalonFX(HoodConstants.HoodMotorPort);
    
    m_Hoodmotor.setInverted(true);
    m_Hoodmotor.setSensorPhase(false);
    m_Hoodmotor.setNeutralMode(NeutralMode.Brake);//???
    //设定pid
    m_Hoodmotor.selectProfileSlot(0, 0);//TODO
    m_Hoodmotor.config_kP(0, 0.5, 10);
    m_Hoodmotor.config_kI(0, 0.0, 10);
    m_Hoodmotor.config_kD(0, 0.0, 10);
    m_Hoodmotor.config_kF(0, 0.00, 10);
    m_Hoodmotor.config_IntegralZone(0, 0, 10);
    m_Hoodmotor.configMotionCruiseVelocity(20000, 10);//???
    m_Hoodmotor.configMotionAcceleration(20000, 10);//???
    
    // mHoodmotor.configMotionSCurveStrength(6);

    m_Hoodmotor.configForwardSoftLimitThreshold(Conversions.degreesToTalon(HoodConstants.HOOD_MAX_ANGLE, HoodConstants.HOOD_GEAR_RATIO) , 10); //TODO
    m_Hoodmotor.configReverseSoftLimitThreshold(Conversions.degreesToTalon(HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_GEAR_RATIO) , 10); //TODO
    m_Hoodmotor.configForwardSoftLimitEnable(true, 10);
    m_Hoodmotor.configReverseSoftLimitEnable(true, 10);

    m_Hoodmotor.configVoltageCompSaturation(10);
    m_Hoodmotor.enableVoltageCompensation(true);

    m_Hoodmotor.configPeakOutputForward(0.50, 10);//TODO
    m_Hoodmotor.configPeakOutputReverse(-0.50, 10);//TODO

    m_Hoodmotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    //mHoodmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
    //mHoodmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
    //mHoodmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
    //mHoodmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);
}
public void SetHoodAngle(double _Angle)
{
  if(!m_Enabled) return;
  if(_Angle>40)
    _Angle=40.;
  m_TargetHoodAngle=_Angle;

  m_Hoodmotor.set(ControlMode.MotionMagic, Conversions.degreesToTalon(_Angle, HoodConstants.HOOD_GEAR_RATIO)/2
  );
}
public double GetHoodAngle()
{
  return Conversions.falconToDegrees(m_Hoodmotor.getSelectedSensorPosition(), HoodConstants.HOOD_GEAR_RATIO);
}
public double GetTargetHoodAngle()
{
  return m_TargetHoodAngle;
}
public boolean IsHoodAtAngle()
{
  return GetTargetHoodAngle()-GetHoodAngle()<=HoodConstants.kHoodTolerance;
}
private void DashboardOutput()
{
  SmartDashboard.putNumber("HoodAnglePresent", GetHoodAngle());
  SmartDashboard.putNumber("TargetHoodAngle", GetTargetHoodAngle());
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
