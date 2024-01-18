// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.PIDTuners;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase.ControlType;


public class FalconTuner{
  TalonFX m_Motor;
  TalonFXConfiguration m_Configuration;
  DeviceIdentifier identifier;
  final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  /** Creates a new FalconTuner. */
  public FalconTuner(int CANID) {
    m_Motor = new TalonFX(CANID);
    m_Configuration = new TalonFXConfiguration();
    
  }

  public void setMotionProfile(double CruiseVelocity, double Accel,double Jerk){
    var mm = m_Configuration.MotionMagic;

    mm.MotionMagicCruiseVelocity = CruiseVelocity;
    mm.MotionMagicAcceleration = Accel;
    mm.MotionMagicJerk = Jerk;


  }

  public void setPIDFGain(double kP, double kI, double kD){
    var slot0 = m_Configuration.Slot0;

    slot0.kP = kP;
    slot0.kI = kI; 
    slot0.kD = kD;
  
  }

  public void setAGSVGain(double kA, double kG, double kS, double kV, GravityTypeValue gravity){
    var slot0 = m_Configuration.Slot0;

    slot0.kA = kA;
    slot0.kG = kG;
    slot0.kS = kS;
    slot0.kV = kV;

    slot0.GravityType = gravity;
  }

  public void applyConfigs(){
    m_Motor.getConfigurator().apply(m_Configuration, 0.050);
  }

  
  public void SetOutput(double Position){
    m_Motor.setControl(m_motionMagic.withPosition(Position));

  
  }
}
