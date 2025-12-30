// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

public class AlignXY extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignXY(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double vx = m_VisionSubsystem.calculateXSpeed();
    double vy = m_VisionSubsystem.calculateYSpeed();
    
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(vx, vy, 0, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
  }

  @Override
  public boolean isFinished() {
    // Command finishes when both X and Y are aligned
    return m_VisionSubsystem.isAlignedX() && m_VisionSubsystem.isAlignedY();
  }
}