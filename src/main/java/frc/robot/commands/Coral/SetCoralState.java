// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.subsystems.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetCoralState extends InstantCommand {
  CoralSubsystem m_CoralSubsystem;
  CoralStates m_CoralState;
  public SetCoralState(CoralSubsystem m_CoralSubsystem, CoralStates m_CoralState) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.m_CoralSubsystem = m_CoralSubsystem;
    this.m_CoralState = m_CoralState;

    addRequirements(m_CoralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CoralSubsystem.setState(m_CoralState);
  }
}
