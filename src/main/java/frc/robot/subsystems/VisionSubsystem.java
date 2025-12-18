package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AlignmentPosition;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight";

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;

    private boolean m_hasValidTarget;
    private double m_tx;
    private double m_ty;
    private double m_targetYaw; // Yaw component from camera-to-tag transform

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;

    public VisionSubsystem() {
        m_xController = new PIDController(
            VisionConstants.kXP, 
            VisionConstants.kXI, 
            VisionConstants.kXD
        );
        
        m_yController = new PIDController(
            VisionConstants.kYP, 
            VisionConstants.kYI, 
            VisionConstants.kYD
        );

        m_rotationController = new PIDController(
            VisionConstants.kRotP,
            VisionConstants.kRotI,
            VisionConstants.kRotD
        );
        
        m_rotationController.enableContinuousInput(-180, 180);
    }

    private void updateTargetData() {
        m_hasValidTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);

        if (m_hasValidTarget) {
            m_tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
            m_ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
            
            // Get camera pose in target space
            double[] camPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace(LIMELIGHT_NAME);
            
            // Array format: [x, y, z, roll, pitch, yaw]
            // Let's try roll (index 3) - this might represent the rotation we care about
            if (camPoseTargetSpace.length >= 6) {
                // Try roll first
                m_targetYaw = camPoseTargetSpace[3]; // Roll
                
                System.out.println("Camera pose target space:");
                System.out.println("  X: " + camPoseTargetSpace[0]);
                System.out.println("  Y: " + camPoseTargetSpace[1]);
                System.out.println("  Z: " + camPoseTargetSpace[2]);
                System.out.println("  Roll: " + camPoseTargetSpace[3]);
                System.out.println("  Pitch: " + camPoseTargetSpace[4]);
                System.out.println("  Yaw: " + camPoseTargetSpace[5]);
                System.out.println("Using ROLL as target angle");
            } else {
                m_targetYaw = 0;
            }
            
        } else {
            m_tx = 0;
            m_ty = 0;
            m_targetYaw = 0;
        }
    }

    public double calculateXSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double targetOffset = m_alignmentPosition.getOffsetMeters();
        double error = m_tx - targetOffset;
        
        if (Math.abs(error) < VisionConstants.kXTolerance) {
            return 0;
        }
        
        return -m_xController.calculate(m_tx, targetOffset);
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double error = m_ty - VisionConstants.kTargetTY;
        
        if (Math.abs(error) < VisionConstants.kYTolerance) {
            return 0;
        }

        return -m_yController.calculate(m_ty, VisionConstants.kTargetTY);
    }

    /**
     * NEW APPROACH: Use camera pose in target space.
     * The roll value tells us our rotation relative to the tag.
     * We want the FRONT of the robot perpendicular to the tag.
     * 
     * IMPORTANT: We add 90° because we want the FRONT perpendicular,
     * not the side. The roll=0 means side is perpendicular, so we offset by 90°.
     */
    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            return 0;
        }

        // We want roll to be -90° (or +90°) for front to be perpendicular
        // Target: -90 degrees
        double targetRoll = -90.0;
        double error = m_targetYaw - targetRoll;
        
        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        SmartDashboard.putNumber("Vision/CurrentRoll", m_targetYaw);
        SmartDashboard.putNumber("Vision/TargetRoll", targetRoll);
        SmartDashboard.putNumber("Vision/RotError", error);
        
        if (Math.abs(error) < VisionConstants.kRotationTolerance) {
            return 0;
        }

        // Rotate to achieve target roll
        double speed = -m_rotationController.calculate(m_targetYaw, targetRoll);
        
        // Clamp the speed
        double maxSpeed = 0.3;
        speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));
        
        SmartDashboard.putNumber("Vision/RotSpeed", speed);
        
        return speed;
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;
        return Math.abs(m_tx - m_alignmentPosition.getOffsetMeters()) < VisionConstants.kXTolerance;
    }

    public boolean isAlignedY() {
        if (!m_hasValidTarget) return false;
        return Math.abs(m_ty - VisionConstants.kTargetTY) < VisionConstants.kYTolerance;
    }

    public boolean isAlignedRotation(double currentRobotHeading) {
        if (!m_hasValidTarget) return false;
        
        double targetRoll = -90.0;
        double error = m_targetYaw - targetRoll;
        
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        return Math.abs(error) < VisionConstants.kRotationTolerance;
    }
    
    public boolean hasValidTarget() {
        return m_hasValidTarget;
    }

    public void setAlignmentPosition(AlignmentPosition position) {
        m_alignmentPosition = position;
    }

    @Override
    public void periodic() {
        updateTargetData();

        SmartDashboard.putBoolean("Vision/HasTarget", m_hasValidTarget);
        SmartDashboard.putNumber("Vision/TX", m_tx);
        SmartDashboard.putNumber("Vision/TY", m_ty);
        SmartDashboard.putNumber("Vision/TargetYaw", m_targetYaw);
        SmartDashboard.putBoolean("Vision/Aligned_X", isAlignedX());
        SmartDashboard.putBoolean("Vision/Aligned_Y", isAlignedY());
    }
}