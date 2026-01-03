package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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

    // Minimum output thresholds to prevent tiny commands
    private static final double MIN_X_OUTPUT = 0.02;
    private static final double MIN_ROT_OUTPUT = 0.03;
    private static final double MIN_Y_OUTPUT = 0.02;

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
        
        m_rotationController.setTolerance(VisionConstants.kRotationTolerance);
        m_xController.setTolerance(VisionConstants.kXTolerance);
        m_yController.setTolerance(VisionConstants.kYTolerance);
    }

    private void updateTargetData() {
        m_hasValidTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);

        if (m_hasValidTarget) {
            m_tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
            m_ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
            
            double[] botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
            
            if (botPoseTargetSpace.length >= 6) {
                m_targetYaw = botPoseTargetSpace[4]; 
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
            m_xController.reset();
            return 0;
        }

        double targetOffset = m_alignmentPosition.getOffsetMeters();
        double error = m_tx - targetOffset;
        
        // Expanded dead band - accept "good enough" to prevent oscillation
        if (Math.abs(error) < VisionConstants.kXTolerance * 1.5) {
            m_xController.reset(); // Clear any accumulated I term
            return 0;
        }
        
        double speed = -m_xController.calculate(m_tx, targetOffset);
        
        // Minimum output threshold - prevent tiny commands that cause jitter
        if (Math.abs(speed) < MIN_X_OUTPUT) {
            m_xController.reset();
            return 0;
        }
        
        // Clamp to reasonable max speed
        speed = MathUtil.clamp(speed, -0.3, 0.3);
        
        SmartDashboard.putNumber("Vision/XError", error);
        SmartDashboard.putNumber("Vision/XSpeed", speed);
        
        return speed;
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            m_yController.reset();
            return 0;
        }

        double error = m_ty - VisionConstants.kTargetTY;
        
        // Expanded dead band
        if (Math.abs(error) < VisionConstants.kYTolerance * 1.5) {
            m_yController.reset();
            return 0;
        }

        double speed = -m_yController.calculate(m_ty, VisionConstants.kTargetTY);
        
        // Minimum output threshold
        if (Math.abs(speed) < MIN_Y_OUTPUT) {
            m_yController.reset();
            return 0;
        }
        
        speed = MathUtil.clamp(speed, -0.3, 0.3);
        
        SmartDashboard.putNumber("Vision/YError", error);
        SmartDashboard.putNumber("Vision/YSpeed", speed);
        
        return speed;
    }

    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            m_rotationController.reset();
            return 0;
        }

        double error = m_targetYaw;
        
        SmartDashboard.putNumber("Vision/RotError", error);
        
        // Wider dead band for rotation - prevents oscillation
        if (Math.abs(error) < VisionConstants.kRotationTolerance * 2.0) {
            m_rotationController.reset();
            return 0;
        }

        double speed = m_rotationController.calculate(error, 0);
        
        // Minimum command threshold - critical for swerve module stability
        if (Math.abs(speed) < MIN_ROT_OUTPUT) {
            m_rotationController.reset();
            return 0;
        }
        
        // Adaptive max speed - slower when close
        double maxSpeed = 0.25;
        if (Math.abs(error) < 5) maxSpeed = 0.15;
        if (Math.abs(error) < 2) maxSpeed = 0.08;
        
        speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
        
        SmartDashboard.putNumber("Vision/RotSpeed", speed);
        
        return speed;
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
        return Math.abs(m_targetYaw) < VisionConstants.kRotationTolerance;
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
        SmartDashboard.putBoolean("Vision/Aligned_Rotation", isAlignedRotation(0));
    }
}