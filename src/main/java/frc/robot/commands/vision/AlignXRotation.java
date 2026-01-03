package frc.robot.commands.vision;

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
  // Increased threshold for more stable completion
  private static final int FINISH_THRESHOLD = 25;  // 0.5 seconds at 50Hz

  public AlignXRotation(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    m_stableCount = 0;
    SmartDashboard.putString("Vision/Command", "AlignXRotation Started");
  }

  @Override
  public void execute() {
    // Check for valid target first
    if (!m_VisionSubsystem.hasValidTarget()) {
      m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
      return;
    }
    
    double vx = m_VisionSubsystem.calculateXSpeed();
    double rotSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    SmartDashboard.putNumber("Vision/CMD_XSpeed", vx);
    SmartDashboard.putNumber("Vision/CMD_RotSpeed", rotSpeed);
    
    // Both speeds at 0 means we're within acceptable range
    if (vx == 0 && rotSpeed == 0) {
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
    // Must have valid target to finish successfully
    if (!m_VisionSubsystem.hasValidTarget()) {
      return false;
    }
    
    boolean aligned = m_VisionSubsystem.isAlignedX() && 
                      m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
    
    // Require sustained alignment to prevent premature completion
    return aligned && m_stableCount >= FINISH_THRESHOLD;
  }
}