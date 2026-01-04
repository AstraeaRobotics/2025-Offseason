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
    private int m_fiducialID;

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;

    private static final double MIN_COMMAND = 0.02;
    
    private static final double[] TAG_TY_TARGETS = {
        1.75,  // Tag 0
        1.82,  // Tag 1
        3.2,   // Tag 2
        0.27,  // Tag 3
        5.44,  // Tag 4
        3.48   // Tag 5
    };

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
            m_fiducialID = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
            
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
            m_fiducialID = -1;
        }
    }

    public double calculateXSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double targetOffset = m_alignmentPosition.getOffsetMeters();
        double error = m_tx - targetOffset;
        
        double speed = -m_xController.calculate(m_tx, targetOffset);
        
        if (Math.abs(speed) < MIN_COMMAND) {
            speed = 0;
        }
        
        double maxSpeed = 0.25;
        if (Math.abs(error) < 2.0) maxSpeed = 0.15;
        if (Math.abs(error) < 1.0) maxSpeed = 0.10;
        
        speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
        
        SmartDashboard.putNumber("Vision/XError", error);
        SmartDashboard.putNumber("Vision/XSpeed_Raw", -m_xController.calculate(m_tx, targetOffset));
        SmartDashboard.putNumber("Vision/XSpeed_Final", speed);
        
        return speed;
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            return 0;
        }

        double targetTY = getTargetTY();
        double error = m_ty - targetTY;
        double speed = -m_yController.calculate(m_ty, targetTY);
        
        if (Math.abs(speed) < MIN_COMMAND) {
            speed = 0;
        }
        
        double maxSpeed = 0.25;
        if (Math.abs(error) < 2.0) maxSpeed = 0.15;
        if (Math.abs(error) < 1.0) maxSpeed = 0.10;
        
        speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
        
        SmartDashboard.putNumber("Vision/YError", error);
        SmartDashboard.putNumber("Vision/YSpeed", speed);
        SmartDashboard.putNumber("Vision/TargetTY", targetTY);
        
        return speed;
    }
    
    private double getTargetTY() {
        if (m_fiducialID >= 0 && m_fiducialID < TAG_TY_TARGETS.length) {
            return TAG_TY_TARGETS[m_fiducialID];
        }
        return VisionConstants.kTargetTY;
    }

    public double calculateRotationSpeed(double currentRobotHeading) {
        if (!m_hasValidTarget) {
            return 0;
        }

        double error = m_targetYaw;
        double speed = m_rotationController.calculate(error, 0);
        
        if (Math.abs(speed) < MIN_COMMAND) {
            speed = 0;
        }
        
        double maxSpeed = 0.20;
        if (Math.abs(error) < 5) maxSpeed = 0.12;
        if (Math.abs(error) < 2) maxSpeed = 0.08;
        
        speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
        
        SmartDashboard.putNumber("Vision/RotError", error);
        SmartDashboard.putNumber("Vision/RotSpeed", speed);
        
        return speed;
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;
        
        double targetOffset = m_alignmentPosition.getOffsetMeters();
        double error = m_tx - targetOffset;
        double rawSpeed = Math.abs(m_xController.calculate(m_tx, targetOffset));
        
        boolean withinTolerance = Math.abs(error) < m_xController.getPositionTolerance();
        boolean commandTooSmall = rawSpeed < MIN_COMMAND;
        
        boolean aligned = withinTolerance || commandTooSmall;
        
        SmartDashboard.putBoolean("Vision/X_WithinTol", withinTolerance);
        SmartDashboard.putBoolean("Vision/X_CmdSmall", commandTooSmall);
        SmartDashboard.putBoolean("Vision/X_Aligned", aligned);
        
        return aligned;
    }

    public boolean isAlignedY() {
        if (!m_hasValidTarget) return false;
        
        double targetTY = getTargetTY();
        double error = m_ty - targetTY;
        double rawSpeed = Math.abs(m_yController.calculate(m_ty, targetTY));
        
        return Math.abs(error) < m_yController.getPositionTolerance() || rawSpeed < MIN_COMMAND;
    }

    public boolean isAlignedRotation(double currentRobotHeading) {
        if (!m_hasValidTarget) return false;
        
        double error = m_targetYaw;
        double rawSpeed = Math.abs(m_rotationController.calculate(error, 0));
        
        boolean aligned = Math.abs(error) < m_rotationController.getPositionTolerance() || rawSpeed < MIN_COMMAND;
        
        SmartDashboard.putBoolean("Vision/Rot_Aligned", aligned);
        
        return aligned;
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
        SmartDashboard.putNumber("Vision/FiducialID", m_fiducialID);
    }
}