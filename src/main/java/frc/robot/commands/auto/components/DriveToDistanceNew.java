// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistanceNew extends Command {
  /** Creates a new DriveToDistanceNew. */

  private final SwerveSubsystem m_swerveSubsystem;
  private final double m_xDistanceMeters;
  private final double m_yDistanceMeters;
  private double initialXPosition;
  private double initialYPosition;
  private double xDriveSpeed;
  private double yDriveSpeed;
  private final double desiredHeading;
  private PIDController rotationController;

  public DriveToDistanceNew(SwerveSubsystem swerveSubsystem, double xDistanceMeters, double yDistanceMeters, double desiredHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_xDistanceMeters = xDistanceMeters;
    this.m_yDistanceMeters = yDistanceMeters;
    this.desiredHeading = desiredHeading;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSubsystem.resetEncoders();

    initialXPosition = m_swerveSubsystem.getPose().getX();
    initialYPosition = m_swerveSubsystem.getPose().getY();
    
    rotationController = new PIDController(0.03, 0, 0);
    rotationController.enableContinuousInput(0, 360);

    double speedMultiplier = 0.5;
    double baseX = 0.05 * Math.abs(m_xDistanceMeters);
    double baseY = 0.05 * Math.abs(m_yDistanceMeters);

    xDriveSpeed = baseX * Math.signum(m_xDistanceMeters) * DrivebaseConstants.kAutoSpeedMultiplier * speedMultiplier;
    yDriveSpeed = baseY * Math.signum(-m_yDistanceMeters) * DrivebaseConstants.kAutoSpeedMultiplier * speedMultiplier;

    SmartDashboard.putNumber("DriveToDistanceNew/Target X (m)", Math.abs(m_xDistanceMeters));
    SmartDashboard.putNumber("DriveToDistanceNew/Target Y (m)", Math.abs(m_yDistanceMeters));
    SmartDashboard.putNumber("DriveToDistanceNew/X Speed", xDriveSpeed);
    SmartDashboard.putNumber("DriveToDistanceNew/Y Speed", yDriveSpeed);
    SmartDashboard.putNumber("DriveToDistanceNew/Initial X Pos", initialXPosition);
    SmartDashboard.putNumber("DriveToDistanceNew/Initial Y Pos", initialYPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(
      (m_xDistanceMeters == 0 ? 0 : xDriveSpeed),
      (m_yDistanceMeters == 0 ? 0 : yDriveSpeed),
      rotationController.calculate(m_swerveSubsystem.getHeading(), desiredHeading),
      m_swerveSubsystem.getHeading()
    ),
    false
    );

    double currentX = Math.abs(getCurrentXDistanceTraveled());
    double currentY = Math.abs(getCurrentYDistanceTraveled());

    SmartDashboard.putNumber("DriveToDistanceNew/Current X (m)", currentX);
    SmartDashboard.putNumber("DriveToDistanceNew/Current Y (m)", currentY);
    SmartDashboard.putNumber("DriveToDistanceNew/X Remaining (m)", Math.abs(m_xDistanceMeters) - currentX);
    SmartDashboard.putNumber("DriveToDistanceNew/Y Remaining (m)", Math.abs(m_yDistanceMeters) - currentY);
    SmartDashboard.putNumber("DriveToDistanceNew/Current Pose X", m_swerveSubsystem.getPose().getX());
    SmartDashboard.putNumber("DriveToDistanceNew/Current Pose Y", m_swerveSubsystem.getPose().getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, 0, 0), false);

    System.out.println("DriveToDistanceNew: Command ended - Final X: " + Math.abs(getCurrentXDistanceTraveled()) +
                       "m, Final Y: " + Math.abs(getCurrentYDistanceTraveled()) + "m");
    SmartDashboard.putNumber("DriveToDistanceNew/Final X (m)", Math.abs(getCurrentXDistanceTraveled()));
    SmartDashboard.putNumber("DriveToDistanceNew/Final Y (m)", Math.abs(getCurrentYDistanceTraveled()));
  }

  private double getCurrentXDistanceTraveled() {
    return m_swerveSubsystem.getPose().getX() - initialXPosition;
  }

  private double getCurrentYDistanceTraveled() {
    return m_swerveSubsystem.getPose().getY() - initialYPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentX = Math.abs(getCurrentXDistanceTraveled());
    double currentY = Math.abs(getCurrentYDistanceTraveled());
    double tolerance = 0.01; // 1cm tolerance

    boolean xFinished = (m_xDistanceMeters == 0) || (currentX >= Math.abs(m_xDistanceMeters) - tolerance);
    boolean yFinished = (m_yDistanceMeters == 0) || (currentY >= Math.abs(m_yDistanceMeters) - tolerance);

    boolean finished = xFinished && yFinished;

    if (finished) {
      System.out.println("DriveToDistanceNew: Command finishing - X: " + currentX + "/" + Math.abs(m_xDistanceMeters) +
                         ", Y: " + currentY + "/" + Math.abs(m_yDistanceMeters));
    }
    return finished;
  }
}
