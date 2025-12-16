// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Two-step alignment process:
 * 1. First aligns X and Y (centers on tag and reaches target distance)
 * 2. Then rotates in place to face the tag squarely
 * 
 * This prevents the tag from going out of view during rotation.
 */
public class AlignThenRotate extends SequentialCommandGroup {
  
  public AlignThenRotate(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, boolean slowMode) {
    addCommands(
      // Step 1: Align X and Y position
      new AlignXY(visionSubsystem, swerveSubsystem, slowMode),
      
      // Step 2: Rotate in place (now that we're centered, we won't lose sight)
      new AlignRotationGyro(visionSubsystem, swerveSubsystem, slowMode)
    );
  }
}