// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  private final double m_targetDistance = 0.305; // 1 ft to stop

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_rotationController;

  private boolean m_hasValidTarget;
  private double m_targetX;
  private double m_targetY;
  private double m_distanceToTarget;
  private double m_angleToTarget;

  public VisionSubsystem() {
    m_xController = new PIDController(2.1, 0, 0);
    m_yController = new PIDController(2.1, 0, 0);
    m_rotationController = new PIDController(2.0, 0, 0.1);

    m_rotationController.enableContinuousInput(-180, 180);

    m_hasValidTarget = false;
    m_targetX = 0;
    m_targetY = 0;
    m_distanceToTarget = 0;
    m_angleToTarget = 0;
  }

  private void updateTargetData() {
    m_hasValidTarget = LimelightHelpers.getTV("limelight");

    if(!m_hasValidTarget) {
      m_targetX = 0;
      m_targetY = 0;
      m_distanceToTarget = 0;
      m_angleToTarget = 0;
      return;
    }

    double[] m_targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");

    if (m_targetPose == null || m_targetPose.length < 6) {
      m_hasValidTarget = false;
      return;
    }

    m_targetX = m_targetPose[0];
    m_targetY = m_targetPose[1];

    m_distanceToTarget = Math.sqrt(Math.pow(m_targetX, 2) + Math.pow(m_targetY, 2));
    m_angleToTarget = Math.toDegrees(Math.atan2(m_targetX, m_targetY));
  }

  public boolean hasTarget() {
    return m_hasValidTarget;
  }

  public double getTargetX() {
    return m_targetX;
  }

  public double getTargetY() {
    return m_targetY;
  }

  public double getDistanceToTarget() {
    return m_distanceToTarget;
  }

  public double getAngleToTarget() {
    return m_angleToTarget;
  }

  public double[] calculateAlignmentSpeeds() {
    if (!m_hasValidTarget) {
      return new double[]{0, 0, 0};
    }

    double m_desiredDistance = m_distanceToTarget - m_targetDistance;

    double m_ratio = m_desiredDistance / m_distanceToTarget;
    double m_desiredX = m_targetX * m_ratio;
    double m_desiredY = m_targetY * m_ratio;

    double m_xError = m_desiredX;
    double m_yError = m_desiredY;
    
    double m_vx = -m_xController.calculate(0, m_xError); 
    double m_vy = -m_yController.calculate(0, m_yError);  
    double m_omega = Math.toRadians(m_rotationController.calculate(m_angleToTarget, 0)); 
    
    // Debug output
    SmartDashboard.putNumber("Vision: X Error (m)", m_xError);
    SmartDashboard.putNumber("Vision: Y Error (m)", m_yError);
    SmartDashboard.putNumber("Vision: Desired Dist (m)", m_desiredDistance);
    SmartDashboard.putNumber("Vision: vx", m_vx);
    SmartDashboard.putNumber("Vision: vy", m_vy);
    SmartDashboard.putNumber("Vision: omega", m_omega);
    
    return new double[]{m_vx, m_vy, m_omega};
  }

  public boolean isAligned() {
    if (!m_hasValidTarget) {
      return false;
  }

    double m_distanceError = Math.abs(m_distanceToTarget - m_targetDistance);
    
    return m_distanceError < 0.05 && Math.abs(m_angleToTarget) < 5;
  }

  public void resetAlignmentControllers() {
    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();
}

  public void setPipeline(int m_pipelineIndex) {
      LimelightHelpers.setPipelineIndex("limelight", m_pipelineIndex);
  }

  public double getCurrentPipeline() {
      return LimelightHelpers.getCurrentPipelineIndex("limelight");
  }

  public void setLEDsOn() {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
  }


  public void setLEDsOff() {
      LimelightHelpers.setLEDMode_ForceOff("limelight");
  }

  public void setLEDsPipelineControl() {
      LimelightHelpers.setLEDMode_PipelineControl("limelight");
  }

  public double getTX() {
      return LimelightHelpers.getTX("limelight");
  }

  public double getTY() {
      return LimelightHelpers.getTY("limelight");
  }

  public double getTA() {
      return LimelightHelpers.getTA("limelight");
  }

  public double getFiducialID() {
      return LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTargetData();
        
    SmartDashboard.putBoolean("Vision: Has Target", m_hasValidTarget);
    SmartDashboard.putNumber("Vision: Target X (m)", m_targetX);
    SmartDashboard.putNumber("Vision: Target Y (m)", m_targetY);
    SmartDashboard.putNumber("Vision: Distance (m)", m_distanceToTarget);
    SmartDashboard.putNumber("Vision: Angle (deg)", m_angleToTarget);
  }
}
