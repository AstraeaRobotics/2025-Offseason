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
    private double m_targetYaw; 

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
            
            double[] botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
            
            SmartDashboard.putNumber("Vision/BotPose_TS_Length", botPoseTargetSpace.length);
            
            if (botPoseTargetSpace.length >= 6) {
                SmartDashboard.putNumber("Vision/BotPose_TS_X", botPoseTargetSpace[0]);
                SmartDashboard.putNumber("Vision/BotPose_TS_Y", botPoseTargetSpace[1]);
                SmartDashboard.putNumber("Vision/BotPose_TS_Z", botPoseTargetSpace[2]);
                SmartDashboard.putNumber("Vision/BotPose_TS_Roll", botPoseTargetSpace[3]);
                SmartDashboard.putNumber("Vision/BotPose_TS_Pitch", botPoseTargetSpace[4]);
                SmartDashboard.putNumber("Vision/BotPose_TS_Yaw", botPoseTargetSpace[5]);
                
                m_targetYaw = botPoseTargetSpace[4]; 
            } else {
                m_targetYaw = 0;
                SmartDashboard.putString("Vision/BotPose_TS_Error", "Array too short");
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

    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            SmartDashboard.putString("Vision/RotDebug", "No Target");
            return 0;
        }

        double error = m_targetYaw;
        
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        SmartDashboard.putNumber("Vision/RotError", error);
        SmartDashboard.putNumber("Vision/RotErrorAbs", Math.abs(error));
        SmartDashboard.putNumber("Vision/RotTolerance", VisionConstants.kRotationTolerance);
        
        if (Math.abs(error) < VisionConstants.kRotationTolerance) {
            SmartDashboard.putString("Vision/RotDebug", "Within Tolerance");
            return 0;
        }

        double speed = m_rotationController.calculate(m_targetYaw, 0);
        
        SmartDashboard.putNumber("Vision/RotSpeedRaw", speed);

        double maxSpeed = 0.3;
        speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));
        
        SmartDashboard.putNumber("Vision/RotSpeedClamped", speed);
        SmartDashboard.putString("Vision/RotDebug", "Rotating - Error: " + String.format("%.2f", error));
        
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
        
        double error = m_targetYaw;
        
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
        SmartDashboard.putNumber("Vision/TargetPitch", m_targetYaw);
        SmartDashboard.putBoolean("Vision/Aligned_X", isAlignedX());
        SmartDashboard.putBoolean("Vision/Aligned_Y", isAlignedY());
        SmartDashboard.putBoolean("Vision/Aligned_Rotation", isAlignedRotation(0));
        SmartDashboard.putNumber("Vision/RotError", m_targetYaw);
    }
}