package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.AlignmentPosition;

public class RobotSuperstructure extends SubsystemBase {
    
    private final SwerveSubsystem m_swerve;
    private final VisionSubsystem m_vision;
    
    private SuperstructureState m_currentState = SuperstructureState.MANUAL_DRIVE;
    private SuperstructureState m_desiredState = SuperstructureState.MANUAL_DRIVE;
    
    private boolean m_isTransitioning = false;
    
    private AlignmentPosition m_targetAlignment = AlignmentPosition.CENTER;

    public enum SuperstructureState {
        MANUAL_DRIVE,       
        VISION_ALIGN_X,     
        VISION_ALIGN_ROTATION,
        VISION_ALIGN_FULL,     
        VISION_DRIVE_TO_TARGET, 
        FIELD_RELATIVE_DRIVE,   
        ROBOT_CENTRIC_NUDGE,   
        AUTO_PATHFINDING,      
        DISABLED                
    }

    public RobotSuperstructure(SwerveSubsystem swerve, VisionSubsystem vision) {
        this.m_swerve = swerve;
        this.m_vision = vision;
    }

    public void setState(SuperstructureState desiredState) {
        if (m_desiredState != desiredState) {
            m_desiredState = desiredState;
            m_isTransitioning = true;
            
            SmartDashboard.putString("Superstructure/Desired State", desiredState.toString());
        }
    }

    public SuperstructureState getCurrentState() {
        return m_currentState;
    }

    public boolean isTransitioning() {
        return m_isTransitioning;
    }

    public boolean isAtDesiredState() {
        return m_currentState == m_desiredState && !m_isTransitioning;
    }

    public void setAlignmentTarget(AlignmentPosition position) {
        m_targetAlignment = position;
        m_vision.setAlignmentPosition(position);
    }

    public AlignmentPosition getAlignmentTarget() {
        return m_targetAlignment;
    }

    public void executeVisionAlignX(boolean slowMode) {
        if (!m_vision.hasValidTarget()) {
            stopMovement();
            return;
        }
        
        double xSpeed = m_vision.calculateXSpeed();
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, 0, 0);
        m_swerve.drive(speeds, slowMode);
    }

    public void executeVisionAlignRotation(boolean slowMode) {
        if (!m_vision.hasValidTarget()) {
            stopMovement();
            return;
        }
        
        double rotSpeed = m_vision.calculateRotationSpeed(m_swerve.getHeading());
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotSpeed);
        m_swerve.drive(speeds, slowMode);
    }

    public void executeVisionAlignFull(boolean slowMode) {
        if (!m_vision.hasValidTarget()) {
            stopMovement();
            return;
        }
        
        double xSpeed = m_vision.calculateXSpeed();
        double rotSpeed = m_vision.calculateRotationSpeed(m_swerve.getHeading());
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, 0, rotSpeed);
        m_swerve.drive(speeds, slowMode);
    }

    public boolean isVisionAlignedX() {
        return m_vision.isAlignedX();
    }

    public boolean isVisionAlignedRotation() {
        return m_vision.isAlignedRotation(m_swerve.getHeading());
    }

    public boolean isVisionAlignedFull() {
        return m_vision.isAlignedX() && m_vision.isAlignedRotation(m_swerve.getHeading());
    }

    public void stopMovement() {
        m_swerve.drive(new ChassisSpeeds(0, 0, 0), false);
    }

    public void emergencyStop() {
        stopMovement();
        m_currentState = SuperstructureState.DISABLED;
        m_desiredState = SuperstructureState.DISABLED;
        m_isTransitioning = false;
        SmartDashboard.putBoolean("Superstructure/Emergency Stop", true);
    }

    public Pose2d getRobotPose() {
        return m_swerve.getPose();
    }

    public void resetPose(Pose2d pose) {
        m_swerve.resetRobotPose(pose);
    }

    public void resetGyro() {
        m_swerve.resetGyro();
    }

    public double getHeading() {
        return m_swerve.getHeading();
    }

    public boolean hasVisionTarget() {
        return m_vision.hasValidTarget();
    }

    public SwerveSubsystem getSwerve() {
        return m_swerve;
    }
    
    public VisionSubsystem getVision() {
        return m_vision;
    }
    
    private void updateStateMachine() {
        if (m_isTransitioning) {
            m_currentState = m_desiredState;
            m_isTransitioning = false;
        }
    }
    
    private void checkSafety() {
        if ((m_currentState == SuperstructureState.VISION_ALIGN_X ||
             m_currentState == SuperstructureState.VISION_ALIGN_ROTATION ||
             m_currentState == SuperstructureState.VISION_ALIGN_FULL ||
             m_currentState == SuperstructureState.VISION_DRIVE_TO_TARGET) &&
            !m_vision.hasValidTarget()) {
            SmartDashboard.putBoolean("Superstructure/Vision Warning", true);
        } 
        
        else {
            SmartDashboard.putBoolean("Superstructure/Vision Warning", false);
        }
        
        boolean isSafe = true; 
        SmartDashboard.putBoolean("Superstructure/Safe", isSafe);
    }
    
    @Override
    public void periodic() {
        updateStateMachine();
        
        checkSafety();
        
        SmartDashboard.putString("Superstructure/Current State", m_currentState.toString());
        SmartDashboard.putString("Superstructure/Desired State", m_desiredState.toString());
        SmartDashboard.putBoolean("Superstructure/Transitioning", m_isTransitioning);
        SmartDashboard.putString("Superstructure/Alignment Target", m_targetAlignment.toString());
        SmartDashboard.putBoolean("Superstructure/Has Vision Target", m_vision.hasValidTarget());

        Pose2d pose = getRobotPose();
        SmartDashboard.putNumber("Superstructure/Robot X", pose.getX());
        SmartDashboard.putNumber("Superstructure/Robot Y", pose.getY());
        SmartDashboard.putNumber("Superstructure/Robot Heading", pose.getRotation().getDegrees());
    }
}