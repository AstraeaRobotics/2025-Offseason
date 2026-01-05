// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturntoAbsoluteHome extends SequentialCommandGroup {
  /** Creates a new ReturnHome. */

  private static final PathConstraints CONSTRAINTS = new PathConstraints(
    0.5, 
    0.4,
    Units.degreesToRadians(90),
    Units.degreesToRadians(180) 
  );

  public ReturntoAbsoluteHome(SwerveSubsystem m_SwerveSubsystem, VisionSubsystem m_VisionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      AutoBuilder.pathfindToPose(
        new Pose2d(-1.07, 1.31, Rotation2d.fromDegrees(0)), 
        CONSTRAINTS,
        0.0
      )
    );
  }
}
