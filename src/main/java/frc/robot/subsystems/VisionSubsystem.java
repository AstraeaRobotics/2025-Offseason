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
    private double m_targetAngle; // The angle we need to face to be perpendicular to tag

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
        
        m_rotationController.enableContinuousInput(0, 360);
        m_targetAngle = 0;
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

    /**
     * Call this when starting rotation alignment to lock in the target angle.
     * This calculates what heading we need to face the tag straight on.
     */
    public void lockTargetAngle(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            m_targetAngle = currentRobotHeading;
            return;
        }
        
        // The target angle is our current heading adjusted by tx
        // If tx is positive (tag is to the right), we need to rotate right
        m_targetAngle = (currentRobotHeading + m_tx + 360) % 360;
        
        SmartDashboard.putNumber("Vision/TargetAngle", m_targetAngle);
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
     * Calculates rotation speed based on the locked target angle and current robot heading.
     * This allows the robot to spin in place without losing sight of the tag.
     */
    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            return 0;
        }

        double error = m_targetAngle - currentRobotHeading;
        
        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        SmartDashboard.putNumber("Vision/RotError", error);
        
        if (Math.abs(error) < VisionConstants.kRotationTolerance) {
            return 0;
        }

        return m_rotationController.calculate(currentRobotHeading, m_targetAngle);
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
        
        double error = m_targetAngle - currentRobotHeading;
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

        // Organized SmartDashboard output
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasValidTarget);
        
        // Current values
        SmartDashboard.putNumber("Vision/TX", m_tx);
        SmartDashboard.putNumber("Vision/TY", m_ty);
        
        // Alignment status
        SmartDashboard.putBoolean("Vision/Aligned_X", isAlignedX());
        SmartDashboard.putBoolean("Vision/Aligned_Y", isAlignedY());
    }
}