// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnWheelsAndDrive extends SequentialCommandGroup {
  /** Creates a new TurnWheelsAndDrive. */
  public TurnWheelsAndDrive(SwerveSubsystem m_SwerveSubsystem, double xDist, double yDist, double desiredHeading) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(0.01), new TurnWheels(m_SwerveSubsystem, xDist, yDist, 0)), new DriveToDistance(m_SwerveSubsystem, xDist, yDist, desiredHeading));
  }
}
