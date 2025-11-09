// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */

  private final SwerveSubsystem m_SwerveSubsystem;
  private final VisionSubsystem m_VisionSubsystem;

  public AutoAlign(SwerveSubsystem m_SwerveSubsystem, VisionSubsystem m_VisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto-align started");
    m_VisionSubsystem.resetAlignmentControllers();
    SmartDashboard.putString("Auto-Align Status", "STARTING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_VisionSubsystem.hasTarget()) {
      m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true);
      SmartDashboard.putString("Auto-Align Status", "NO TARGET");
      return;
  }

  double[] speeds = m_VisionSubsystem.calculateAlignmentSpeeds();
  double vx = speeds[0];    
  double vy = speeds[1];  
  double omega = speeds[2]; 
  
  m_SwerveSubsystem.drive(new ChassisSpeeds(vx, vy, omega), true);
  
  SmartDashboard.putString("Auto-Align Status", "ALIGNING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true);
    
    if (interrupted) {
        System.out.println("Auto-align interrupted");
        SmartDashboard.putString("Auto-Align Status", "INTERRUPTED");
    } 
    
    else {
        System.out.println("Auto-align complete");
        SmartDashboard.putString("Auto-Align Status", "ALIGNED");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.isAligned();
  }
}
