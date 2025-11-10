// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.coral.*;
import frc.robot.commands.auto.paths.L1Mid;
import frc.robot.commands.auto.paths.LL1Side;
import frc.robot.commands.auto.paths.LL2Side;
import frc.robot.commands.auto.paths.RL1Side;
import frc.robot.commands.auto.paths.RL2Side;
import frc.robot.commands.elevator.IncrementSetpoint;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.swerve.DriveRobotCentric;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.TeleopSwerveNEW;
import frc.robot.commands.vision.AutoAlign;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // Joystick Controllers - Port 0: Left stick (forward/back), Port 1: Right stick (strafe/rotation)
  private final Joystick m_leftJoystick = new Joystick(0);
  private final Joystick m_rightJoystick = new Joystick(1);

  private final PS4Controller m_Controller = new PS4Controller(2);
  public static final GenericHID operatorGamepad = new GenericHID(3);

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

  private final JoystickButton kCircle = new JoystickButton(m_Controller, PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_Controller, PS4Controller.Button.kSquare.value);
  private final JoystickButton kCross = new JoystickButton(m_Controller, PS4Controller.Button.kCross.value);
  private final JoystickButton kTriangle = new JoystickButton(m_Controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kR1 = new JoystickButton(m_Controller, PS4Controller.Button.kR1.value);
  private final JoystickButton kR2 = new JoystickButton(m_Controller, PS4Controller.Button.kR2.value);
  private final JoystickButton kL1 = new JoystickButton(m_Controller, PS4Controller.Button.kL1.value);
  private final JoystickButton kL2 = new JoystickButton(m_Controller, PS4Controller.Button.kL2.value);

  private final POVButton pov0 = new POVButton(m_Controller, 0);
  private final POVButton pov90 = new POVButton(m_Controller, 90);
  private final POVButton pov180 = new POVButton(m_Controller, 180);
  private final POVButton pov270 = new POVButton(m_Controller, 270);

  SendableChooser<Command> chooser = new SendableChooser<>();

  private final Command m_L1Mid = new L1Mid(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_RL1Side = new RL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_LL1Side = new LL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_RL2Side = new RL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_LL2Side = new LL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  
  private boolean isSlowModeOn = false; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("RaiseToL1", 
      new ParallelCommandGroup(
        new SetCoralState(m_coralSubsystem, CoralStates.kL1),
        new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1)
      ));
  
    NamedCommands.registerCommand("RaiseToL2", 
      new ParallelCommandGroup(
        new SetCoralState(m_coralSubsystem, CoralStates.kL2),
        new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL2)
      ));
  
    NamedCommands.registerCommand("ScoreCoralL1", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.0),
        new ExtakeL1(m_coralSubsystem)
      ));
  
    NamedCommands.registerCommand("ScoreCoral", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.0),
        new IntakeCoral(m_coralSubsystem, 5)
      ));

    NamedCommands.registerCommand("IntakeCoral",
      new ParallelCommandGroup(
        new WaitCommand(5),
        new IntakeCoral(m_coralSubsystem, -5)
      ));

    chooser.setDefaultOption("L1 Mid (OLD)", m_L1Mid);
    chooser.addOption("RL1 Side (OLD)", m_RL1Side);
    chooser.addOption("LL1 Side (OLD)", m_LL1Side);
    chooser.addOption("RL2 Side (OLD)", m_RL2Side);
    chooser.addOption("LL2Side (OLD)", m_LL2Side);
  
    chooser.addOption("L1Mid", AutoBuilder.buildAuto("L1Mid"));
    chooser.addOption("RL2", AutoBuilder.buildAuto("RL2"));
    chooser.addOption("RL1", AutoBuilder.buildAuto("RL1"));
    chooser.addOption("LL2", AutoBuilder.buildAuto("LL2"));
    chooser.addOption("LL1", AutoBuilder.buildAuto("LL1"));

    chooser.addOption("2Pc_RL1", AutoBuilder.buildAuto("2Pc_RL1"));
  
    SmartDashboard.putData("Auto choices", chooser);

    // Left joystick Y-axis: forward/backward, Right joystick X-axis: strafe, Right joystick Z-axis: rotation
    m_SwerveSubsystem.setDefaultCommand(new TeleopSwerveNEW(
      m_SwerveSubsystem,
      () -> -m_rightJoystick.getX(),
      () -> -m_leftJoystick.getY(),
      () -> -m_rightJoystick.getZ(),
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

    kCross.onTrue(new ResetGyro(m_SwerveSubsystem));
    kR1.whileTrue(new IntakeCoral(m_coralSubsystem, -5));
    kL1.whileTrue(new IntakeCoral(m_coralSubsystem, 5));
    kTriangle.whileTrue(new ExtakeL1(m_coralSubsystem));

    kCircle.onTrue(new AutoAlign(m_SwerveSubsystem, m_VisionSubsystem));

    kSquare.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
    }));

    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));

    kOperator1.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // R
    kOperator2.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kSource), new SetCoralState(m_coralSubsystem, CoralStates.kSource))); // SRC
    kOperator3.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1), new SetCoralState(m_coralSubsystem, CoralStates.kL1))); // CL1
    kOperator4.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL2), new SetCoralState(m_coralSubsystem, CoralStates.kL2))); // Cl2
    kOperator5.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL3), new SetCoralState(m_coralSubsystem, CoralStates.kL3))); // Cl3
    kOperator9.onTrue(new IncrementSetpoint(m_ElevatorSubsystem, 1)); // IL
    kOperator10.onTrue(new IncrementSetpoint(m_ElevatorSubsystem, -1)); // DL
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}