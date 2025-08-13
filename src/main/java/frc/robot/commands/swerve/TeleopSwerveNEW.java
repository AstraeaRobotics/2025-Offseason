// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;

public class TeleopSwerveNEW extends Command {
  /** Creates a new TeleopSwerve. */
  SwerveSubsystem m_SwerveSubsystem;
  
  DoubleSupplier m_driveX;
  DoubleSupplier m_driveY;
  DoubleSupplier m_rotation;

  private final BooleanSupplier slowModeSupplier;

  public TeleopSwerveNEW(SwerveSubsystem swerveSub, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotation, BooleanSupplier slowModeSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveX = driveX;
    m_driveY = driveY;
    m_rotation = rotation;
    m_SwerveSubsystem = swerveSub;
    this.slowModeSupplier = slowModeSupplier;

    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    boolean slowMode = slowModeSupplier.getAsBoolean();

    if (Math.abs(m_driveX.getAsDouble()) > 0.2 || Math.abs(m_driveY.getAsDouble()) > 0.2) {
      m_SwerveSubsystem.drive(
        SwerveUtil.driveInputToChassisSpeeds(
          m_driveX.getAsDouble(), m_driveY.getAsDouble(), m_rotation.getAsDouble(), m_SwerveSubsystem.getHeading()),
        slowMode);
    } 
    
    else if (Math.abs(m_rotation.getAsDouble()) > 0.2) {
      m_SwerveSubsystem.drive(
        SwerveUtil.driveInputToChassisSpeeds(
          0, 0, m_rotation.getAsDouble(), m_SwerveSubsystem.getHeading()),
        slowMode);
    } 
    
    else {
      m_SwerveSubsystem.drive(
        SwerveUtil.driveInputToChassisSpeeds(0, 0, 0, m_SwerveSubsystem.getHeading()),
        slowMode);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
