// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */

  private final SwerveSubsystem m_SwerveSubsystem;
  private final VisionSubsystem m_VisionSubsystem;
  private final AlignmentPosition m_alignmentPosition;
  
  private double startTime = 0;
  private static final double TIMEOUT_SECONDS = 3.0;

  public AutoAlign(SwerveSubsystem m_SwerveSubsystem, VisionSubsystem m_VisionSubsystem, AlignmentPosition m_alignmentPosition) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_alignmentPosition = m_alignmentPosition;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  public AutoAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this(swerveSubsystem, visionSubsystem, null);
  }

  @Override
  public void initialize() {
    if (m_alignmentPosition != null) {
      m_VisionSubsystem.setAlignmentPosition(m_alignmentPosition);
    }
    
    startTime = Timer.getFPGATimestamp();
    System.out.println("Auto-align started - Position: " + m_VisionSubsystem.getAlignmentPosition());
    m_VisionSubsystem.resetAlignmentControllers();
    SmartDashboard.putString("Auto-Align Status", "STARTING");
  }

  @Override
  public void execute() {
    if (!m_VisionSubsystem.hasTarget()) {
      m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true);
      SmartDashboard.putString("Auto-Align Status", "NO TARGET");
      return;
    }

    double[] speeds = m_VisionSubsystem.calculateAlignmentSpeeds();
    double vx = speeds[0]; 
    double vy = speeds[1];  
    double omega = speeds[2]; 
    
    omega = Math.toRadians(omega);
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vy, vx, omega);
    
    m_SwerveSubsystem.drive(chassisSpeeds, true);
    
    SmartDashboard.putString("Auto-Align Status", "ALIGNING");
    SmartDashboard.putNumber("AutoAlign vx", vx);
    SmartDashboard.putNumber("AutoAlign vy", vy);
    SmartDashboard.putNumber("AutoAlign omega", Math.toDegrees(omega));
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true);
    
    if (interrupted) {
      System.out.println("Auto-align interrupted");
      SmartDashboard.putString("Auto-Align Status", "INTERRUPTED");
    } 
    else {
      System.out.println("Auto-align complete - " + m_VisionSubsystem.getAlignmentPosition());
      SmartDashboard.putString("Auto-Align Status", "ALIGNED");
    }
  }

  @Override
  public boolean isFinished() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    
    if (elapsedTime > TIMEOUT_SECONDS) {
      System.out.println("Auto-align timed out after " + elapsedTime + " seconds");
      SmartDashboard.putString("Auto-Align Status", "TIMEOUT");
      return true;
    }
    
    boolean aligned = m_VisionSubsystem.isAligned();
    SmartDashboard.putBoolean("Vision: Is Aligned", aligned);
    SmartDashboard.putNumber("AutoAlign Time Elapsed", elapsedTime);
    
    return aligned;
  }
}