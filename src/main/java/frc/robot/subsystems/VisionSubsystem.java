// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.*;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  
  private final double LIMELIGHT_TO_FRONT_OFFSET = 0.338; 
  
  private final double DESIRED_FRONT_DISTANCE = 0.0254; // 1in 
  
  private final double m_targetDistance = DESIRED_FRONT_DISTANCE + LIMELIGHT_TO_FRONT_OFFSET; 

  final double X_KP = 0.006;
  final double X_KI = 0;
  final double X_KD = 0;
  
  final double Y_KP = 0.008; // may need to tune
  final double Y_KI = 0;
  final double Y_KD = 0;
  
  // may need to tune
  final double ROT_KP = 2.0;
  final double ROT_KI = 0;
  final double ROT_KD = 0.1;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_rotationController;

  private boolean m_hasValidTarget;
  private double m_targetX;
  private double m_targetY;
  private double m_distanceToTarget;
  private double m_angleToTarget;

  private AlignmentPosition m_alignmentPosition;
  
  private int alignedFrameCount = 0;
  private static final int REQUIRED_ALIGNED_FRAMES = 10;

  public VisionSubsystem() {
    m_xController = new PIDController(X_KP, X_KI, X_KD);  
    m_yController = new PIDController(Y_KP, Y_KI, Y_KD);  
    m_rotationController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    m_rotationController.enableContinuousInput(-180, 180);

    m_hasValidTarget = false;
    m_targetX = 0;
    m_targetY = 0;
    m_distanceToTarget = 0;
    m_angleToTarget = 0;
    m_alignmentPosition = AlignmentPosition.CENTER; 

    SmartDashboard.putNumber("Vision X_KP", X_KP);
    SmartDashboard.putNumber("Vision X_KD", X_KD);
    SmartDashboard.putNumber("Vision Y_KP", Y_KP);
    SmartDashboard.putNumber("Vision Y_KP", Y_KD);
    SmartDashboard.putNumber("Vision ROT_KP", ROT_KP);
    SmartDashboard.putNumber("Vision ROT_KP", ROT_KD);
  }

  public void updatePIDLive() {
    double xKp = SmartDashboard.getNumber("Vision X_KP", X_KP);
    double xKd = SmartDashboard.getNumber("Vision X_KD", X_KD);
    double yKp = SmartDashboard.getNumber("Vision Y_KP", Y_KP);
    double yKd = SmartDashboard.getNumber("Vision Y_KP", Y_KD);
    double rotKp = SmartDashboard.getNumber("Vision ROT_KP", ROT_KP);
    double rotKd = SmartDashboard.getNumber("Vision ROT_KP", ROT_KD);

    m_xController.setPID(xKp,0, xKd);
    m_yController.setPID(yKp,0, yKd);
    m_rotationController.setPID(rotKp,0, rotKd);
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
  
  public void setAlignmentPosition(AlignmentPosition position) {
    m_alignmentPosition = position;
  }
  
  public AlignmentPosition getAlignmentPosition() {
    return m_alignmentPosition;
  }

  public double[] calculateAlignmentSpeeds() {
    if (!m_hasValidTarget) {
      return new double[]{0, 0, 0};
    }

    double m_desiredDistance = m_distanceToTarget - m_targetDistance;

    double m_ratio = m_desiredDistance / m_distanceToTarget;
    
    double m_desiredX = m_targetX * m_ratio;
    double m_desiredY = m_targetY * m_ratio;
    
    double horizontalOffset = m_alignmentPosition.getOffsetMeters();
    m_desiredX += horizontalOffset;
    
    double m_xError = m_desiredX;
    double m_yError = m_desiredY;
    
    double m_vx = m_xController.calculate(0, m_xError);  
    double m_vy = m_yController.calculate(0, m_yError);  
    double m_omega = m_rotationController.calculate(m_angleToTarget, 0); 
    
    SmartDashboard.putNumber("Vision: X Error (m)", m_xError);
    SmartDashboard.putNumber("Vision: Y Error (m)", m_yError);
    SmartDashboard.putNumber("Vision: Angle Error (deg)", m_angleToTarget);
    SmartDashboard.putNumber("Vision: Desired Dist (m)", m_desiredDistance);
    SmartDashboard.putNumber("Vision: vx (m/s)", m_vx);
    SmartDashboard.putNumber("Vision: vy (m/s)", m_vy);
    SmartDashboard.putNumber("Vision: omega (deg/s)", m_omega);
    SmartDashboard.putString("Vision: Alignment", m_alignmentPosition.name());
    
    return new double[]{m_vx, m_vy, m_omega};
  }

  public boolean isAligned() {
    if (!m_hasValidTarget) {
      alignedFrameCount = 0;
      return false;
    }

    double m_distanceError = Math.abs(m_distanceToTarget - m_targetDistance);
    double horizontalError = Math.abs(m_targetX - m_alignmentPosition.getOffsetMeters());
    
    boolean distanceOK = m_distanceError < 0.08; 
    boolean angleOK = Math.abs(m_angleToTarget) < 8;  
    boolean horizontalOK = horizontalError < 0.08;  
    
    SmartDashboard.putBoolean("Vision: Distance OK", distanceOK);
    SmartDashboard.putBoolean("Vision: Angle OK", angleOK);
    SmartDashboard.putBoolean("Vision: Horizontal OK", horizontalOK);
    SmartDashboard.putNumber("Vision: Aligned Frame Count", alignedFrameCount);
    
    boolean currentlyAligned = distanceOK && angleOK && horizontalOK;
    
    if (currentlyAligned) {
      alignedFrameCount++;
    } 
    
    else {
      alignedFrameCount = 0;
    }
    
    return alignedFrameCount >= REQUIRED_ALIGNED_FRAMES;
  }

  public void resetAlignmentControllers() {
    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();
    alignedFrameCount = 0;
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
    updateTargetData();
        
    SmartDashboard.putBoolean("Vision: Has Target", m_hasValidTarget);
    SmartDashboard.putNumber("Vision: Target X (m)", m_targetX);
    SmartDashboard.putNumber("Vision: Target Y (m)", m_targetY);
    SmartDashboard.putNumber("Vision: Distance (m)", m_distanceToTarget);
    SmartDashboard.putNumber("Vision: Angle (deg)", m_angleToTarget);

    updatePIDLive();
  }
}