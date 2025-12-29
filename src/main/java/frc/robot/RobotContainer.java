package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.swerve.DriveRobotCentric;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.TeleopSwerveNEW;
import frc.robot.commands.vision.AlignX;
import frc.robot.commands.vision.AlignXRotation;
import frc.robot.commands.vision.AlignY;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // LIMELIGHT NAME - Match this with VisionSubsystem
  private static final String LIMELIGHT_NAME = "limelight";
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  private final XboxController m_Controller = new XboxController(0);
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

  private final JoystickButton kB = new JoystickButton(m_Controller, XboxController.Button.kB.value);
  private final JoystickButton kX = new JoystickButton(m_Controller, XboxController.Button.kX.value);
  private final JoystickButton kA = new JoystickButton(m_Controller, XboxController.Button.kA.value);
  private final JoystickButton kY = new JoystickButton(m_Controller, XboxController.Button.kY.value);
  private final JoystickButton kRightBumper = new JoystickButton(m_Controller, XboxController.Button.kRightBumper.value);
  private final JoystickButton kLeftBumper = new JoystickButton(m_Controller, XboxController.Button.kLeftBumper.value);

  private final POVButton pov0 = new POVButton(m_Controller, 0);
  private final POVButton pov90 = new POVButton(m_Controller, 90);
  private final POVButton pov180 = new POVButton(m_Controller, 180);
  private final POVButton pov270 = new POVButton(m_Controller, 270);
  
  private boolean isSlowModeOn = false; 

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

  private void configureBindings() {
    kA.onTrue(new ResetGyro(m_SwerveSubsystem));

    kX.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
      SmartDashboard.putBoolean("SlowMode", isSlowModeOn);
    }));

    kB.onTrue(new AlignX(m_VisionSubsystem, m_SwerveSubsystem, true));
    kY.onTrue(new AlignY(m_VisionSubsystem, m_SwerveSubsystem, true));
    
    kLeftBumper.onTrue(new AlignXRotation(m_VisionSubsystem, m_SwerveSubsystem, true));

    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}