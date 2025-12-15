// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

public class AlignY extends Command {
  /** Creates a new AlignY. */

  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignY(VisionSubsystem m_VisionSubsystem, SwerveSubsystem m_SwerveSubsystem, boolean m_slowMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_slowMode = m_slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vy = m_VisionSubsystem.calculateYSpeed();
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(0, vy, 0, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.isAlignedY();
  }
}