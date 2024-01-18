// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// current limelight id is "limelight-maxxi"
public class Limelight extends SubsystemBase {

  public NetworkTableInstance limelightInstance;
  public NetworkTable m_limelight;
  private SwerveDrivePoseEstimator m_poseEstimator;

    public Limelight(String p_NetworkTableID, SwerveDrivePoseEstimator p_PoseEstimator) {
    this.limelightInstance = NetworkTableInstance.getDefault();
    this.m_limelight = limelightInstance.getTable(p_NetworkTableID);
    this.m_poseEstimator = p_PoseEstimator;
  }

  public static enum m_ledStates{
    leftOn,
    rightOn,
    on,
    off
  }
  
  public void setLight(m_ledStates p_state){
    NetworkTableEntry m_ledEntry = m_limelight.getEntry("ledMode");
    double m_ledMode = 0;

    switch(p_state){
      case leftOn:
        m_ledMode = 4;
        System.out.println("L");
        break;
      case rightOn:
        m_ledMode = 2;
        System.out.println("R");
        break;
      case on:
        m_ledMode = 3;
        System.out.println("ON");
        break;
      case off:
        m_ledMode = 0;
        System.out.println("OFF");
        break;
    }
    m_ledEntry.setDouble(m_ledMode);

  }

  public double[] getTarget(){
    NetworkTableEntry m_txEntry = m_limelight.getEntry("tx");
    NetworkTableEntry m_tyEntry = m_limelight.getEntry("ty");
    double tx = m_txEntry.getDouble(0);
    double ty = m_tyEntry.getDouble(0);

    return new double[] {tx, ty};
  }

  public Pose2d getApriltag(){
    NetworkTableEntry pose = m_limelight.getEntry("botpose_wpiblue");
    double[] array = pose.getDoubleArray(new double[8]);
    return new Pose2d(new Translation2d(array[0], array[1]), new Rotation2d().fromDegrees(array[5]));

  }

  public double getTimestamp(){
    NetworkTableEntry timeStampEntry = m_limelight.getEntry("botpose_wpiblue");
    double[] error = timeStampEntry.getDoubleArray(new double[8]);
    return Timer.getFPGATimestamp() - (error[6]/1000.0);
  }

  public void updatePoseWithAprilTag(){
    if (getApriltag().getX() > 0.1 && getApriltag().getY() > 0.1){
    m_poseEstimator.addVisionMeasurement(getApriltag(), getTimestamp());
    }
  }


 
  /** Creates a new Limelight. */


  @Override
  public void periodic() {
    updatePoseWithAprilTag();
    // This method will be called once per scheduler run
  }
}
