package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.autos.DriveToTag;
import frc.robot.autos.ReturnHome;
import frc.robot.commands.swerve.DriveRobotCentric;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.TeleopSwerveNEW;
import frc.robot.commands.vision.AlignFull;
import frc.robot.commands.vision.AlignX;
import frc.robot.commands.vision.AlignXRotation;
import frc.robot.commands.vision.AlignY;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private static final String LIMELIGHT_NAME = "limelight";
  
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

  private final JoystickButton kCircle = new JoystickButton(m_Controller, PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_Controller, PS4Controller.Button.kSquare.value);
  private final JoystickButton kCross = new JoystickButton(m_Controller, PS4Controller.Button.kCross.value);
  private final JoystickButton kTriangle = new JoystickButton(m_Controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kR1 = new JoystickButton(m_Controller, PS4Controller.Button.kR1.value);
  private final JoystickButton kL1 = new JoystickButton(m_Controller, PS4Controller.Button.kL1.value);
  private final JoystickButton kR2 = new JoystickButton(m_Controller, PS4Controller.Button.kR2.value);
  private final JoystickButton kL2 = new JoystickButton(m_Controller, PS4Controller.Button.kL2.value);
  
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
    kCross.onTrue(new ResetGyro(m_SwerveSubsystem));

    kSquare.onTrue(new InstantCommand(() -> {
      isSlowModeOn = !isSlowModeOn;
      SmartDashboard.putBoolean("SlowMode", isSlowModeOn);
    }));

    kCircle.onTrue(new AlignX(m_VisionSubsystem, m_SwerveSubsystem, true));
    
    kR1.onTrue(new AlignY(m_VisionSubsystem, m_SwerveSubsystem, true));

    kL1.onTrue(new AlignXRotation(m_VisionSubsystem, m_SwerveSubsystem, true));

    kL2.onTrue(new AlignFull(m_VisionSubsystem, m_SwerveSubsystem, true));

    kOperator1.onTrue(new ReturnHome(m_SwerveSubsystem, m_VisionSubsystem));

    //tag alignment
    kOperator2.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,0));
    kOperator3.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,1));
    kOperator4.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,2));
    kOperator5.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,3));
    kOperator6.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,4));
    kOperator7.onTrue(new DriveToTag(m_SwerveSubsystem, m_VisionSubsystem,5));

    //robot centric moving
    pov0.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, -DrivebaseConstants.kRobotCentricVel, 0));
    pov180.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, DrivebaseConstants.kRobotCentricVel, 0));
    pov270.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, -DrivebaseConstants.kRobotCentricVel));
    pov90.whileTrue(new DriveRobotCentric(m_SwerveSubsystem, 0, DrivebaseConstants.kRobotCentricVel));
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
    return null;
  }
}