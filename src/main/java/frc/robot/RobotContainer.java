package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.VisionConstants.AlignmentPosition;
import frc.robot.commands.swerve.DriveRobotCentric;
import frc.robot.commands.swerve.TeleopSwerveNEW;
import frc.robot.commands.swerve_vision_autos.DriveToPoseAlign;
import frc.robot.commands.superstructure.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  private final RobotSuperstructure m_Superstructure = new RobotSuperstructure(
      m_SwerveSubsystem,
      m_VisionSubsystem
  );

  private final XboxController m_driverController = new XboxController(0);
  public static final GenericHID m_operatorController = new GenericHID(1);

  private final JoystickButton kB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton kX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton kA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton kY = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton kRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton kLeftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton kStart = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton kBack = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

  public static final JoystickButton kOperator1 = new JoystickButton(m_operatorController, 1);
  public static final JoystickButton kOperator2 = new JoystickButton(m_operatorController, 2);
  public static final JoystickButton kOperator3 = new JoystickButton(m_operatorController, 3);
  public static final JoystickButton kOperator4 = new JoystickButton(m_operatorController, 4);

  private final POVButton pov0 = new POVButton(m_driverController, 0);
  private final POVButton pov90 = new POVButton(m_driverController, 90);
  private final POVButton pov180 = new POVButton(m_driverController, 180);
  private final POVButton pov270 = new POVButton(m_driverController, 270);
  
  private boolean isSlowModeOn = false;

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(new TeleopSwerveNEW(
      m_SwerveSubsystem,
      m_driverController::getLeftX,
      m_driverController::getLeftY,
      m_driverController::getRightX,
      () -> isSlowModeOn  
    ));

    configureBindings();

    m_autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configureBindings() {

    kA.onTrue(new ResetGyroSuper(m_Superstructure));

    kX.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
      SmartDashboard.putBoolean("SlowMode", isSlowModeOn);
      SmartDashboard.putString("Drive Mode", isSlowModeOn ? "SLOW" : "NORMAL");
    }));

    kB.onTrue(new VisionAlignX(m_Superstructure, true));
    
    kY.onTrue(new VisionAlignFull(m_Superstructure, true)); 

    kLeftBumper.onTrue(new VisionAlignRotation(m_Superstructure, true));

    kRightBumper.onTrue(new DriveToPoseAlign(m_SwerveSubsystem, m_VisionSubsystem));

    kStart.and(kBack).onTrue(new EmergencyStopCommand(m_Superstructure));

    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));

    kOperator1.onTrue(new SetAlignmentTarget(m_Superstructure, AlignmentPosition.CENTER)
        .andThen(new InstantCommand(() -> 
            SmartDashboard.putString("Alignment", "CENTER"))));
    
    kOperator2.onTrue(new SetAlignmentTarget(m_Superstructure, AlignmentPosition.LEFT_EDGE)
        .andThen(new InstantCommand(() -> 
            SmartDashboard.putString("Alignment", "LEFT"))));
    
    kOperator3.onTrue(new SetAlignmentTarget(m_Superstructure, AlignmentPosition.RIGHT_EDGE)
        .andThen(new InstantCommand(() -> 
            SmartDashboard.putString("Alignment", "RIGHT"))));
    
    kOperator4.onTrue(
        new VisionAlignFull(m_Superstructure, true)
            .andThen(new InstantCommand(() -> 
                SmartDashboard.putString("Status", "Aligned!"))));
  }

  private SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();

    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    
    return chooser;
  }

  public void registerNamedCommands() {
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "AutoAlignCENTER",
        new SetAlignmentTarget(m_Superstructure, AlignmentPosition.CENTER)
            .andThen(new VisionAlignFull(m_Superstructure, true))
    );
    
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "AutoAlignLEFT",
        new SetAlignmentTarget(m_Superstructure, AlignmentPosition.LEFT_EDGE)
            .andThen(new VisionAlignFull(m_Superstructure, true))
    );
    
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "AutoAlignRIGHT",
        new SetAlignmentTarget(m_Superstructure, AlignmentPosition.RIGHT_EDGE)
            .andThen(new VisionAlignFull(m_Superstructure, true))
    );
  }

  public Command driveToPoint(double x, double y, double rotationDegrees) {
    return AutoBuilder.pathfindToPose(
        new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees)),
        new PathConstraints(
            .4,
            .5,
            Units.degreesToRadians(90), 
            Units.degreesToRadians(180)  
        ),
        0.0 
    );
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public RobotSuperstructure getSuperstructure() {
    return m_Superstructure;
  }
}