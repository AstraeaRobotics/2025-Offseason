// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

/**
 * Rotates the robot to face the AprilTag squarely.
 * Uses the robot's gyro and locks in the target angle at the start,
 * so the robot spins in place without losing sight of the tag.
 */
public class AlignRotationGyro extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignRotationGyro(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_slowMode = slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    // Lock in the target angle based on current position and tx
    m_VisionSubsystem.lockTargetAngle(m_SwerveSubsystem.getHeading());
    SmartDashboard.putString("Vision/Status", "Rotating...");
  }

  @Override
  public void execute() {
    // Get rotation speed based on gyro heading vs target angle
    double rotationSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    // Create chassis speeds with only rotation
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(0, 0, rotationSpeed, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
    
    SmartDashboard.putNumber("Vision/CurrentHeading", m_SwerveSubsystem.getHeading());
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    SmartDashboard.putString("Vision/Status", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
  }
}