package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class SwerveSubsystem extends SubsystemBase {
  
  SwerveDriveKinematics kinematics;
  AHRS gyro;

  SwerveModule[] swerveModules;

  Translation2d m_frontLeftLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2); 

  SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  StructPublisher<Pose2d> publisher;

  private final Field2d m_field = new Field2d();
  RobotConfig config;

  public double translationalKP = 2.1; 
  public double rotationalKP = 2.0; 
  public double rotationalKD = 0.1; 

  // PID Controllers for auto-align (moved to VisionSubsystem)
  // Removed - vision logic now in VisionSubsystem

  public SwerveSubsystem() {
    kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    gyro = new AHRS();

    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(12, 11, 0, "front left", true);
    swerveModules[1] = new SwerveModule(14, 13, 0, "front right", true);
    swerveModules[2] = new SwerveModule(16, 15, 0, "back left", true);
    swerveModules[3] = new SwerveModule(18, 17, 0, "back right", true);
    
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      Rotation2d.fromDegrees(getHeading()),
      getModulePositions(),
      new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0))
    );  
    
    publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

    SmartDashboard.putData("Field", m_field);
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetRobotPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> drive(speeds, true), 
      new PPHolonomicDriveController(new PIDConstants(2.1, 0, 0), new PIDConstants(2.0, 0, 0.1)), 
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
    
    gyro.reset();
  }

  public void drive(ChassisSpeeds speeds, boolean slowMode) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    for(int i = 0; i < swerveModuleStates.length; i++){
      swerveModules[i].setState(swerveModuleStates[i], slowMode);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getModulePosition();
    }
    
    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getHeading() {
    return (gyro.getYaw() + 360) % 360;
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetGyro() {
    gyro.reset();
    resetRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
  }

  public void resetRobotPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), pose);
  }

  public void resetEncoders() {
    for(int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].resetEncoder();
    }
  }

  public double getEncoderPosition() {
    return swerveModules[0].getDistance();
  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getHeading()), getModulePositions());
    
    publisher.set(getPose());
    
    m_field.setRobotPose(getPose());

    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getPose().getRotation().getDegrees());

    
  }
}