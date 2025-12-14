package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.AlignmentPosition;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    private final double LIMELIGHT_TO_FRONT_OFFSET = 0.338;
    private final double DESIRED_FRONT_DISTANCE = 0.15;
    private final double m_targetDistance = DESIRED_FRONT_DISTANCE + LIMELIGHT_TO_FRONT_OFFSET;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;

    private boolean m_hasValidTarget;
    private double m_targetX;
    private double m_targetY;
    private double m_distanceToTarget;
    private double m_angleToTarget;

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;
    private int alignedFrameCount = 0;
    private static final int REQUIRED_ALIGNED_FRAMES = 10;

    private boolean limelightAvailable = false;

    public VisionSubsystem() {
        m_xController = new PIDController(0.006, 0, 0);
        m_yController = new PIDController(0, 0, 0);
        m_rotationController = new PIDController(0, 0, 0);
        m_rotationController.enableContinuousInput(-180, 180);

        limelightAvailable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").exists();
    }

    private void updateTargetData() {
        if (!limelightAvailable) {
            m_hasValidTarget = false;
            m_targetX = 0;
            m_targetY = 0;
            m_distanceToTarget = 0;
            m_angleToTarget = 0;
            return;
        }

        m_hasValidTarget = LimelightHelpers.getTV("limelight");

        if (!m_hasValidTarget) {
            m_targetX = 0;
            m_targetY = 0;
            m_distanceToTarget = 0;
            m_angleToTarget = 0;
            return;
        }

        double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");

        if (pose == null || pose.length < 6) {
            m_hasValidTarget = false;
            return;
        }

        m_targetX = pose[0];
        m_targetY = pose[1];
        m_distanceToTarget = Math.sqrt(m_targetX * m_targetX + m_targetY * m_targetY);
        m_angleToTarget = Math.toDegrees(Math.atan2(m_targetX, m_targetY));
    }

    public double calculateXSpeed() {
        if (!m_hasValidTarget) return 0;

        double horizontalOffset = m_alignmentPosition.getOffsetMeters();
        m_xController.setSetpoint(horizontalOffset);

        return m_xController.calculate(m_targetX);
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;

        double horizontalError = Math.abs(m_targetX - m_alignmentPosition.getOffsetMeters());
        return horizontalError < 0.08;
    }

    @Override
    public void periodic() {
        updateTargetData();

        SmartDashboard.putBoolean("Vision: Limelight Connected", limelightAvailable);
        SmartDashboard.putBoolean("Vision: Has Target", m_hasValidTarget);
        SmartDashboard.putNumber("Vision: X", m_targetX);
        SmartDashboard.putNumber("Vision: Y", m_targetY);
    }
}
