// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.PIDTuners;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK3.driveRatios;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class NeoTuner{
  /** Creates a new NeoTuner. */
  CANSparkMax m_Motor; 
  SparkPIDController m_PidController;
  RelativeEncoder m_Encoder;


  public NeoTuner(int CANID) {
    m_Motor = new CANSparkMax(CANID, MotorType.kBrushless);

    m_PidController = m_Motor.getPIDController();

    m_Encoder = m_Motor.getEncoder();
  }

  public void setMotionProfile(double MaxVel, double MinVel,double MaxAcc,double AllowedError){
    m_PidController.setSmartMotionMaxVelocity(MaxVel, 0);
    m_PidController.setSmartMotionMinOutputVelocity(MinVel, 0);
    m_PidController.setSmartMotionMaxAccel(MaxAcc, 0);
    m_PidController.setSmartMotionAllowedClosedLoopError(AllowedError, 0);
  }

  public void setPIDFGain(double kP, double kI, double kD, double kF){
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setFF(kF);
    
  }

  public void setPIDFGain(double kP, double kI, double kD, double kF, double OutputRangeMin, double OutputRangeMax){
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setFF(kF);
    m_PidController.setOutputRange(OutputRangeMin, OutputRangeMax);
  }
  
  public void SetOutput(double Output){
    m_PidController.setReference(Output, ControlType.kSmartMotion);
  }
}
