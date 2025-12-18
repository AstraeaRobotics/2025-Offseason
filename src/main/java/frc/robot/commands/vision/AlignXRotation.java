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
 * Aligns X (strafe) and Rotation simultaneously.
 * This keeps the tag in view while rotating to be parallel with it.
 * 
 * Think of it as: robot strafes sideways while spinning to stay centered on tag
 * and end up parallel to the tag's face.
 */
public class AlignXRotation extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignXRotation(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("=== STARTING X + ROTATION ALIGN ===");
  }

  @Override
  public void execute() {
    // Calculate both X strafe and rotation simultaneously
    double vx = m_VisionSubsystem.calculateXSpeed();
    double rotationSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    // Combine both - this keeps tag centered while we rotate to be parallel
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(vx, 0, rotationSpeed, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    System.out.println("=== X + ROTATION ALIGN COMPLETE ===");
  }

  @Override
  public boolean isFinished() {
    // Finishes when both X and rotation are aligned
    return m_VisionSubsystem.isAlignedX() && m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
  }
}