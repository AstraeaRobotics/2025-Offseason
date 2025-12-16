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

    private boolean m_hasValidTarget;
    private double m_tx;
    private double m_ty;

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;

    public VisionSubsystem() {
        // Initialize PID controllers with separate values from Constants
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
    }

    private void updateTargetData() {
        m_hasValidTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);

        if (m_hasValidTarget) {
            m_tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
            m_ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        } else {
            m_tx = 0;
            m_ty = 0;
        }
    }

    public double calculateXSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double targetOffset = m_alignmentPosition.getOffsetMeters();
        double error = m_tx - targetOffset;
        
        // Add deadband to prevent oscillation
        if (Math.abs(error) < VisionConstants.kOffsetTolerance) {
            return 0;
        }
        
        return -m_xController.calculate(m_tx, targetOffset);
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double error = m_ty - VisionConstants.kTargetTY;
        
        // Add deadband to prevent oscillation
        if (Math.abs(error) < VisionConstants.kOffsetTolerance) {
            return 0;
        }

        return -m_yController.calculate(m_ty, VisionConstants.kTargetTY);
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;
        return Math.abs(m_tx - m_alignmentPosition.getOffsetMeters()) < VisionConstants.kOffsetTolerance;
    }

    public boolean isAlignedY() {
        if (!m_hasValidTarget) return false;
        return Math.abs(m_ty - VisionConstants.kTargetTY) < VisionConstants.kOffsetTolerance;
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

        // Clean SmartDashboard output - no nested folders
        SmartDashboard.putBoolean("Vision_HasTarget", m_hasValidTarget);
        SmartDashboard.putNumber("Vision_TX_Current", m_tx);
        SmartDashboard.putNumber("Vision_TY_Current", m_ty);
        SmartDashboard.putNumber("Vision_TX_Target", m_alignmentPosition.getOffsetMeters());
        SmartDashboard.putNumber("Vision_TY_Target", VisionConstants.kTargetTY);
        SmartDashboard.putBoolean("Vision_AlignedX", isAlignedX());
        SmartDashboard.putBoolean("Vision_AlignedY", isAlignedY());
        SmartDashboard.putNumber("Vision_XSpeed", calculateXSpeed());
        SmartDashboard.putNumber("Vision_YSpeed", calculateYSpeed());
    }
}