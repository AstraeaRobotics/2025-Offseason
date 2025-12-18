// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

/**
 * Full alignment - X, Y, and Rotation all at once.
 * The robot will simultaneously:
 * - Strafe to center the tag horizontally (X)
 * - Drive forward/backward to reach target distance (Y) 
 * - Rotate to be parallel with the tag (Rotation)
 */
public class AlignFull extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignFull(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Calculate all three components
    double vx = m_VisionSubsystem.calculateXSpeed();
    double vy = m_VisionSubsystem.calculateYSpeed();
    double rotationSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    // Combine all three movements
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(vx, vy, rotationSpeed, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
  }

  @Override
  public boolean isFinished() {
    // Finishes when ALL three axes are aligned
    return m_VisionSubsystem.isAlignedX() 
        && m_VisionSubsystem.isAlignedY() 
        && m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
  }
}