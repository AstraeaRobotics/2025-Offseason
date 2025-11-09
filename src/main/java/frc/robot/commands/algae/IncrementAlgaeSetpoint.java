// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.algae;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.AlgaeSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class IncrementAlgaeSetpoint extends InstantCommand {
//   AlgaeSubsystem m_AlgaeSubsystem;
//   double incVal;
//   public IncrementAlgaeSetpoint(AlgaeSubsystem m_AlgaeSubsystem, double incVal) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.m_AlgaeSubsystem = m_AlgaeSubsystem;
//     this.incVal = incVal;

//     addRequirements(m_AlgaeSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     this.m_AlgaeSubsystem.desiredSetpoint += incVal;
//   }
// }
