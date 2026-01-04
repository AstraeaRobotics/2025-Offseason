// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignY extends Command {
  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;
  
  private int m_finishCounter = 0;
  private static final int FINISH_THRESHOLD = 10; // 0.2 seconds at 50Hz

  public AlignY(VisionSubsystem m_VisionSubsystem, SwerveSubsystem m_SwerveSubsystem, boolean m_slowMode) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_slowMode = m_slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    m_finishCounter = 0;
    SmartDashboard.putString("Vision/Command", "Y Align Started");
  }

  @Override
  public void execute() {
    double vy = m_VisionSubsystem.calculateYSpeed();
    SmartDashboard.putNumber("Vision/CMD_YSpeed", vy);
    
    // Use robot-relative speeds directly - positive X is forward
    ChassisSpeeds speeds = new ChassisSpeeds(vy, 0, 0);
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
    
    if (m_VisionSubsystem.isAlignedY()) {
      m_finishCounter++;
    } else {
      m_finishCounter = 0;
    }
    
    SmartDashboard.putNumber("Vision/Y_FinishCounter", m_finishCounter);
    
    return m_finishCounter >= FINISH_THRESHOLD;
  }
}