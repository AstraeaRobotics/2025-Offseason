// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class TurnToAngle extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_angle;
  private PIDController controller = new PIDController(.014, 0, .0000325);;
  private Debouncer debouncer = new Debouncer(.15, DebounceType.kBoth);;

  public TurnToAngle(SwerveSubsystem swerveSubsystem, double angle) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_angle = angle;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    double setpoint = m_swerveSubsystem.getHeading() + m_angle + 180;

    controller.setSetpoint(setpoint);
    controller.enableContinuousInput(0, 360);
    controller.setTolerance(1.2);
  }

  @Override
  public void execute() {
    double calculatedRotationSpeed = controller.calculate(m_swerveSubsystem.getHeading() + 180);
    this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, calculatedRotationSpeed, m_swerveSubsystem.getHeading()), false);
  }

  @Override
  public void end(boolean interrupted) {
    this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, 0, m_swerveSubsystem.getHeading()), false);
  }

  @Override
  public boolean isFinished() {
    return debouncer.calculate(controller.atSetpoint());
  }
}