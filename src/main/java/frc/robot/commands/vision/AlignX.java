package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignX extends Command {
  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;
  
  private int m_finishCounter = 0;
  private static final int FINISH_THRESHOLD = 10; // 0.2 seconds at 50Hz

  public AlignX(VisionSubsystem m_VisionSubsystem, SwerveSubsystem m_SwerveSubsystem, boolean m_slowMode) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_slowMode = m_slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    m_finishCounter = 0;
    SmartDashboard.putString("Vision/Command", "X Align Started");
  }

  @Override
  public void execute() {
    double vx = m_VisionSubsystem.calculateXSpeed();
    SmartDashboard.putNumber("Vision/CMD_XSpeed", vx);
    
    ChassisSpeeds speeds = new ChassisSpeeds(0, -vx, 0);
    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    SmartDashboard.putString("Vision/Command", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    if (!m_VisionSubsystem.hasValidTarget()) {
      return false;
    }
    
    if (m_VisionSubsystem.isAlignedX()) {
      m_finishCounter++;
    } else {
      m_finishCounter = 0;
    }
    
    SmartDashboard.putNumber("Vision/FinishCounter", m_finishCounter);
    
    // Must be aligned for 10 consecutive cycles (0.2 sec)
    return m_finishCounter >= FINISH_THRESHOLD;
  }
}