package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

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
    SmartDashboard.putString("Vision/Command", "X+Rotation Align Started");
  }

  @Override
  public void execute() {
    double vx = m_VisionSubsystem.calculateXSpeed();
    double rotationSpeed = m_VisionSubsystem.calculateRotationSpeed(m_SwerveSubsystem.getHeading());
    
    SmartDashboard.putNumber("Vision/XSpeed", vx);
    SmartDashboard.putNumber("Vision/RotSpeed", rotationSpeed);
    
    ChassisSpeeds speeds = SwerveUtil.driveInputToChassisSpeeds(vx, 0, rotationSpeed, m_SwerveSubsystem.getHeading());

    m_SwerveSubsystem.drive(speeds, m_slowMode);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), m_slowMode);
    SmartDashboard.putString("Vision/Command", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    return m_VisionSubsystem.isAlignedX() && m_VisionSubsystem.isAlignedRotation(m_SwerveSubsystem.getHeading());
  }
}