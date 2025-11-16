// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingAlign extends Command {
  /** Creates a new HeadingAlign. */

  private final SwerveSubsystem m_SwerveSubsystem;
  private final VisionSubsystem m_VisionSubsystem;

  private final PIDController m_rotationController;

  private double startTime = 0;

  private static final double timeout = 3.0; // seconds

  private static final double angleTolerance = 2; // degrees

  // # of consec frames needed to be aligned before done
  private int alignedFrameCount = 0;
  private static final int reqAlignFrames = 10;

  public HeadingAlign(SwerveSubsystem m_SwerveSubsystem, VisionSubsystem m_VisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;

    m_rotationController = new PIDController(2.0, 0, 0.1);

    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(angleTolerance);

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    alignedFrameCount = 0;
    m_rotationController.reset();

    System.out.println("Started Align");
    SmartDashboard.putString("Heading Align Status", "STARTING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_VisionSubsystem.hasTarget()) {
      m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true);
      SmartDashboard.putString("Heading Align Status", "NO TARGET");
      return;
    }

    double targetX = m_VisionSubsystem.getTargetX();
    double targetY = m_VisionSubsystem.getTargetY();

    double angleError = Math.toDegrees(Math.atan2(targetY, targetX));

    double omega = m_rotationController.calculate(angleError, 0);

    omega = Math.toRadians(omega);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omega);
    m_SwerveSubsystem.drive(chassisSpeeds,true);

    SmartDashboard.putString("Heading Align Status", "ALIGNING");
    SmartDashboard.putNumber("Heading: Angle Error (deg)", angleError);
    SmartDashboard.putNumber("Heading: Omega Output (deg/s)", Math.toDegrees(omega));
    SmartDashboard.putNumber("Heading: Aligned Frames", alignedFrameCount);
    SmartDashboard.putNumber("Heading:Target X", targetX);
    SmartDashboard.putNumber("Heading:Target Y", targetY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0,0 ), true);

    if (interrupted) {
      System.out.println("Heading align interrupted");
      SmartDashboard.putString("Heading Align Status", "INTERRUPTED");
    }

    else {
      System.out.println("Heading align complete");
      SmartDashboard.putString("Heading Align Status", "COMPLETE");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;

    if (elapsedTime > timeout) {
      System.out.println("Heading align timed out after " + elapsedTime + " seconds");
      SmartDashboard.putString("Heading Align Status", "TIMEOUT");
      return true;
    }

    if (!m_VisionSubsystem.hasTarget()) {
      alignedFrameCount = 0;
      return false;
    }

    double targetX = m_VisionSubsystem.getTargetX();
    double targetY = m_VisionSubsystem.getTargetY();

    double angleError = Math.abs(Math.toDegrees(Math.atan2(targetY, targetX)));

    boolean currentlyAligned = angleError < angleTolerance;

    if (currentlyAligned) {
      alignedFrameCount++;
    }

    else {
      alignedFrameCount = 0;
    }

    SmartDashboard.putBoolean("Heading: Is Aligned", currentlyAligned);

    return alignedFrameCount >= reqAlignFrames;
  }
}
