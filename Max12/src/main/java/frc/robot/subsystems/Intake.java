// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  private CANSparkMax m_topRoller;
  private CANSparkMax m_bottomRoller;


  /** Creates a new Intake. */
  public Intake() {
    m_topRoller = new CANSparkMax(IntakeConstants.IDs.topRollerID, MotorType.kBrushless);
    m_bottomRoller = new CANSparkMax(IntakeConstants.IDs.bottomRollerID, MotorType.kBrushless);

    m_topRoller.setInverted(IntakeConstants.Misc.topRollerIsInverted);
    m_bottomRoller.setInverted(IntakeConstants.Misc.bottomRollerIsInverted);
  }

  public void setIntakeSpeeds(double topSpeed, double bottomSpeed){
    m_topRoller.set(topSpeed);
    m_bottomRoller.set(bottomSpeed);
  }

  public void runIntake(){
    setIntakeSpeeds(IntakeConstants.Speeds.topRollerPercentOut, IntakeConstants.Speeds.bottomRollerPercentOut);
  }

  public void stopIntake(){
    setIntakeSpeeds(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
