// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.swerve.DriveRobotCentric;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.TeleopSwerveNEW;
import frc.robot.commands.vision.AlignX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  private final PS4Controller m_Controller = new PS4Controller(0);
  public static final GenericHID operatorGamepad = new GenericHID(1);

  public static final JoystickButton kOperator1 = new JoystickButton(operatorGamepad, 1);
  public static final JoystickButton kOperator2 = new JoystickButton(operatorGamepad, 2);
  public static final JoystickButton kOperator3 = new JoystickButton(operatorGamepad, 3);
  public static final JoystickButton kOperator4 = new JoystickButton(operatorGamepad, 4);
  public static final JoystickButton kOperator5 = new JoystickButton(operatorGamepad, 5);
  public static final JoystickButton kOperator6 = new JoystickButton(operatorGamepad, 6);
  public static final JoystickButton kOperator7 = new JoystickButton(operatorGamepad, 7);
  public static final JoystickButton kOperator8 = new JoystickButton(operatorGamepad, 8);
  public static final JoystickButton kOperator9 = new JoystickButton(operatorGamepad, 9);
  public static final JoystickButton kOperator10 = new JoystickButton(operatorGamepad, 10);
  public static final JoystickButton kOperator11 = new JoystickButton(operatorGamepad, 11);
  public static final JoystickButton kOperator12 = new JoystickButton(operatorGamepad, 12);

  private final JoystickButton kCircle = new JoystickButton(m_Controller,PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_Controller, PS4Controller.Button.kSquare.value);
  private final JoystickButton kCross = new JoystickButton(m_Controller, PS4Controller.Button.kCross.value);
  private final JoystickButton kTriangle = new JoystickButton(m_Controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kR1 = new JoystickButton(m_Controller,PS4Controller.Button.kR1.value);
  private final JoystickButton kR2 = new JoystickButton(m_Controller,PS4Controller.Button.kR2.value);
  private final JoystickButton kL1 = new JoystickButton(m_Controller,PS4Controller.Button.kL1.value);
  private final JoystickButton kL2 = new JoystickButton(m_Controller,PS4Controller.Button.kL2.value);

  private final POVButton pov0 = new POVButton(m_Controller, 0);
  private final POVButton pov90 = new POVButton(m_Controller, 90);
  private final POVButton pov180 = new POVButton(m_Controller, 180);
  private final POVButton pov270 = new POVButton(m_Controller, 270);
  
  private boolean isSlowModeOn = false; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_SwerveSubsystem.setDefaultCommand(new TeleopSwerveNEW(
      m_SwerveSubsystem,
      m_Controller::getLeftX,
      m_Controller::getLeftY,
      m_Controller::getRightX,
      () -> isSlowModeOn  
    ));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Controller bindings
    kCross.onTrue(new ResetGyro(m_SwerveSubsystem));

    kSquare.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
    }));

    kCircle.onTrue(new AlignX(m_VisionSubsystem, m_SwerveSubsystem, true));

    //robot centric
    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}