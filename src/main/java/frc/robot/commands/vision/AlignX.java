package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

public class AlignX extends Command {
  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private final boolean m_slowMode;

  public AlignX(VisionSubsystem m_VisionSubsystem, SwerveSubsystem m_SwerveSubsystem, boolean m_slowMode) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_slowMode = m_slowMode;

    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Vision/Command", "X Align Started");
  }

  @Override
  public void execute() {
    double vx = m_VisionSubsystem.calculateXSpeed();
    SmartDashboard.putNumber("Vision/XSpeed", vx);
    
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(vx, 0, 0, m_SwerveSubsystem.getHeading());
    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    SmartDashboard.putString("Vision/Command", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.isAlignedX();
  }
}