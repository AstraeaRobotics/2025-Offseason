// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.Coral.*;
import frc.robot.commands.algae.IncrementAlgaeSetpoint;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.commands.algae.SetAlgaeState;
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
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
  private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  // private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

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

  SendableChooser<Command> chooser = new SendableChooser<>();

  private final Command m_L1Mid = new L1Mid(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_RL1Side = new RL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_LL1Side = new LL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_RL2Side = new RL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
  private final Command m_LL2Side = new LL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);

  private boolean isSlowModeOn = false; 

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chooser.setDefaultOption("L1 Mid", m_L1Mid);
    chooser.addOption("RL1 Side", m_RL1Side);
    chooser.addOption("LL1 Side", m_LL1Side);
    chooser.addOption("RL2 Side", m_RL2Side);
    chooser.addOption("LL2Side", m_LL2Side);

    SmartDashboard.putData("Auto choices", chooser);

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
    kR1.whileTrue(new IntakeCoral(m_coralSubsystem, -5));
    kL1.whileTrue(new IntakeCoral(m_coralSubsystem, 5));
    kTriangle.whileTrue(new ExtakeL1(m_coralSubsystem));
    kR2.whileTrue(new IntakeAlgae(m_AlgaeSubsystem, -5));
    kL2.whileTrue(new IntakeAlgae(m_AlgaeSubsystem, 5));
    // kSquare.whileTrue(new SpinClimbMotor(m_ClimbSubsystem, -4));
    // kCircle.whileTrue(new SpinClimbMotor(m_ClimbSubsystem, 4));

    kSquare.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
    }));

    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));

    // Operator gamepad bindings 

    kOperator1.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest), new SetCoralState(m_coralSubsystem, CoralStates.kRest), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // R
    kOperator2.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kSource), new SetCoralState(m_coralSubsystem, CoralStates.kSource), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // SRC
    kOperator3.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1), new SetCoralState(m_coralSubsystem, CoralStates.kL1), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // CL1
    kOperator4.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL2), new SetCoralState(m_coralSubsystem, CoralStates.kL2), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // Cl2
    kOperator5.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL3), new SetCoralState(m_coralSubsystem, CoralStates.kL3), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // Cl3
    kOperator6.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kAl1), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kL1), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // AL1
    kOperator7.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kAL2), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kL2), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // AL2
    kOperator8.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kAl3), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kL3), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // AL3
    // kOperator9.whileTrue(new AlignX(m_SwerveSubsystem, VisionConstants.kLeftOffset)); // AL
    // kOperator10.whileTrue(new AlignX(m_SwerveSubsystem, VisionConstants.kRightOffset)); // AR
    kOperator9.onTrue(new IncrementSetpoint(m_ElevatorSubsystem, 1)); // IL
    kOperator10.onTrue(new IncrementSetpoint(m_ElevatorSubsystem, -1)); // DL
    kOperator11.onTrue(new IncrementAlgaeSetpoint(m_AlgaeSubsystem, 0.1));
    kOperator12.onTrue(new IncrementAlgaeSetpoint(m_AlgaeSubsystem, -0.1));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
    // return new L1Mid(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
    // return new LL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
    // return new LL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
    // return new RL1Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
    // return new RL2Side(m_SwerveSubsystem, m_coralSubsystem, m_ElevatorSubsystem);
    
  }
}
