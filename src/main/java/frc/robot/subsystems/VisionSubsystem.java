package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    
    // Add slew rate limiter to smooth rotation commands
    private final SlewRateLimiter m_rotationLimiter;

    private boolean m_hasValidTarget;
    private double m_tx;
    private double m_ty;
    private double m_targetYaw; // Pitch from botpose_targetspace

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;
    
    // Track previous rotation speed to detect oscillation
    private double m_lastRotationSpeed = 0;
    private int m_oscillationCount = 0;

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
        
        // Don't use continuous input since we're working with pitch
        // which shouldn't wrap around
        m_rotationController.setTolerance(VisionConstants.kRotationTolerance);
        
        // Limit rotation acceleration - prevents jerky movements
        m_rotationLimiter = new SlewRateLimiter(2.5); // units per second max change
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
                
                // Use pitch as the rotation error
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
        
        double speed = -m_xController.calculate(m_tx, targetOffset);
        
        // Clamp to prevent excessive speed
        return MathUtil.clamp(speed, -0.4, 0.4);
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double error = m_ty - VisionConstants.kTargetTY;
        
        if (Math.abs(error) < VisionConstants.kYTolerance) {
            return 0;
        }

        double speed = -m_yController.calculate(m_ty, VisionConstants.kTargetTY);
        return MathUtil.clamp(speed, -0.4, 0.4);
    }

    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            SmartDashboard.putString("Vision/RotDebug", "No Target");
            m_rotationLimiter.reset(0);
            m_lastRotationSpeed = 0;
            return 0;
        }

        double error = m_targetYaw;
        
        SmartDashboard.putNumber("Vision/RotError", error);
        SmartDashboard.putNumber("Vision/RotErrorAbs", Math.abs(error));
        SmartDashboard.putNumber("Vision/RotTolerance", VisionConstants.kRotationTolerance);
        
        if (Math.abs(error) < VisionConstants.kRotationTolerance * 1.5) {
            SmartDashboard.putString("Vision/RotDebug", "Within Tolerance");
            m_rotationLimiter.reset(0);
            m_lastRotationSpeed = 0;
            m_oscillationCount = 0;
            return 0;
        }

        double speed = m_rotationController.calculate(error, 0);
        
        SmartDashboard.putNumber("Vision/RotSpeedRaw", speed);

        if (Math.signum(speed) != Math.signum(m_lastRotationSpeed) && Math.abs(m_lastRotationSpeed) > 0.05) {
            m_oscillationCount++;
            SmartDashboard.putNumber("Vision/OscillationCount", m_oscillationCount);
            
            if (m_oscillationCount > 2) {
                speed *= 0.4;
                SmartDashboard.putString("Vision/RotDebug", "OSCILLATION DETECTED");
            }
        } else if (Math.abs(speed - m_lastRotationSpeed) < 0.02) {
            m_oscillationCount = Math.max(0, m_oscillationCount - 1);
        }
        
        m_lastRotationSpeed = speed;

        double limitedSpeed = m_rotationLimiter.calculate(speed);
        
        double maxSpeed = 0.22;
        
        if (Math.abs(error) < 8) {
            maxSpeed = 0.16;
        }
        if (Math.abs(error) < 4) {
            maxSpeed = 0.10;
        }
        if (Math.abs(error) < 2) {
            maxSpeed = 0.06;
        }
        
        double clampedSpeed = MathUtil.clamp(limitedSpeed, -maxSpeed, maxSpeed);
        
        if (Math.abs(clampedSpeed) > 0 && Math.abs(clampedSpeed) < 0.025) {
            clampedSpeed = Math.signum(clampedSpeed) * 0.025;
        }
        
        SmartDashboard.putNumber("Vision/RotSpeedLimited", limitedSpeed);
        SmartDashboard.putNumber("Vision/RotSpeedClamped", clampedSpeed);
        SmartDashboard.putString("Vision/RotDebug", "Rotating - Error: " + String.format("%.2f", error));
        
        return clampedSpeed;
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;
        double error = m_tx - m_alignmentPosition.getOffsetMeters();
        return Math.abs(error) < VisionConstants.kXTolerance;
    }

    public boolean isAlignedY() {
        if (!m_hasValidTarget) return false;
        double error = m_ty - VisionConstants.kTargetTY;
        return Math.abs(error) < VisionConstants.kYTolerance;
    }

    public boolean isAlignedRotation(double currentRobotHeading) {
        if (!m_hasValidTarget) return false;
        
        double error = m_targetYaw;
        
        // Tighter tolerance - within 1 degree
        return Math.abs(error) < 1.0;
    }
    
    /**
     * Check if rotation is "good enough" to start X alignment.
     * More lenient than isAlignedRotation() to allow moving to next phase.
     */
    public boolean isRotationCloseEnough() {
        if (!m_hasValidTarget) return false;
        double error = m_targetYaw;
        // Within 5 degrees is good enough to start strafing
        return Math.abs(error) < 5.0;
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