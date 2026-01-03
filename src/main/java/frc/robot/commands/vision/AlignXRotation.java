package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignXRotation extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;
  
  private int m_stableCount = 0;

  public AlignXRotation(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    m_stableCount = 0;
    SmartDashboard.putString("Vision/Command", "X+Rotation Align Started");
  }

  @Override
  public void execute() {
    double rawVx = m_VisionSubsystem.calculateXSpeed();
    double rawRotSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    double rotError = Math.abs(SmartDashboard.getNumber("Vision/RotError", 0));
    double xError = Math.abs(SmartDashboard.getNumber("Vision/TX", 0));
    
    double rotationPriority = MathUtil.clamp(rotError / 5.0, 0.0, 1.0);
    double strafePriority = MathUtil.clamp(xError / 5.0, 0.0, 1.0);
    
    double vx = rawVx * strafePriority;
    double rotSpeed = rawRotSpeed * rotationPriority;
    
    if (rotError > 2.0) {
        vx *= 0.3;
    }
    
    if (xError > 2.0) {
        rotSpeed *= 0.5;
    }
    
    if (rotError < 1.5) {
      rotSpeed *= 0.6;
    }
    
    if (xError < 1.0) {
      vx *= 0.6;
    }
    
    SmartDashboard.putNumber("Vision/XSpeed", vx);
    SmartDashboard.putNumber("Vision/RotSpeed", rotSpeed);
    SmartDashboard.putNumber("Vision/RotationPriority", rotationPriority);
    SmartDashboard.putNumber("Vision/StrafePriority", strafePriority);
    SmartDashboard.putNumber("Vision/RotError", rotError);
    SmartDashboard.putNumber("Vision/XError", xError);
    
    boolean rotAligned = m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
    boolean xAligned = m_VisionSubsystem.isAlignedX();
    
    if (rotAligned && xAligned) {
      m_stableCount++;
    } else {
      m_stableCount = 0;
    }
    
    SmartDashboard.putNumber("Vision/StableCount", m_stableCount);
    
    ChassisSpeeds speeds = new ChassisSpeeds(0, -vx, -rotSpeed);
    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    SmartDashboard.putString("Vision/Command", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    boolean tightlyAligned = m_VisionSubsystem.isAlignedX() && 
                             m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
    
    return tightlyAligned && m_stableCount >= 25;
  }
}