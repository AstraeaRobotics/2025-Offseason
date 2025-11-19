// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignX extends Command {
  /** Creates a new AlignX. */

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  public AlignX(SwerveSubsystem swerve, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.vision = vision;

    addRequirements(swerve, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.resetAlignmentControllers();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = vision.calculateAlignmentSpeeds();
    double vx = speeds[0];
    swerve.drive(
      SwerveUtil.driveInputToChassisSpeeds(vx, 0, 0,0),
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(SwerveUtil.driveInputToChassisSpeeds(0, 0, 0, 0),
      false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.isAligned();
  }
}
