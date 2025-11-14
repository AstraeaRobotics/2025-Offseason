// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnWheels extends Command {
  /** Creates a new TurnWheels. */
  SwerveSubsystem m_SwerveSubsystem;
  double driveX; 
  double driveY;
  double rotation;

  public TurnWheels(SwerveSubsystem m_SwerveSubsystem, double driveX, double driveY, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.driveX = driveX;
    this.driveY = driveY;
    this.rotation = rotation;

    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(driveX, driveY, rotation, m_SwerveSubsystem.getHeading()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, 0, m_SwerveSubsystem.getHeading()), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
