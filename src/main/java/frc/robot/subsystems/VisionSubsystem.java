package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.AlignmentPosition;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight";

    private final PIDController m_xController;
    private final PIDController m_yController;

    private boolean m_hasValidTarget;
    private double m_tx; // Horizontal offset in degrees
    private double m_ty; // Vertical offset in degrees

    private AlignmentPosition m_alignmentPosition = AlignmentPosition.CENTER;

    private NetworkTable limelightTable;

    public VisionSubsystem() {
        m_xController = new PIDController(0.0098, 0, 0.0);
        
        m_yController = new PIDController(0.012, 0, 0.0);
        
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        
        System.out.println("===========================================");
        System.out.println("Vision Subsystem Initialized");
        System.out.println("Limelight Name: " + LIMELIGHT_NAME);
        System.out.println("===========================================");
    }

    private void updateTargetData() {
        // Get TV (target valid) directly from NetworkTables
        double tvValue = limelightTable.getEntry("tv").getDouble(0.0);
        m_hasValidTarget = (tvValue == 1.0);

        if (!m_hasValidTarget) {
            m_tx = 0;
            m_ty = 0;
            return;
        }

        // Get TX and TY in degrees
        m_tx = limelightTable.getEntry("tx").getDouble(0.0);
        m_ty = limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double calculateXSpeed() {
        if (!m_hasValidTarget) {
            System.out.println("No valid target for X alignment");
            return 0;
        }

        // Target offset based on alignment position
        double targetOffset = 0.0; // Center alignment
        
        // Calculate speed based on horizontal offset
        double speed = -m_xController.calculate(m_tx, targetOffset);
        
        System.out.println("TX: " + m_tx + ", X Speed: " + speed);
        return speed;
    }

    public double calculateYSpeed() {
        if (!m_hasValidTarget) {
            System.out.println("No valid target for Y alignment");
            return 0;
        }

        double targetTY = 15.35; 

        double speed = -m_yController.calculate(m_ty, targetTY);
        
        System.out.println("TY: " + m_ty + ", Y Speed: " + speed);
        return speed;
    }

    public boolean isAlignedX() {
        if (!m_hasValidTarget) return false;

        // Consider aligned if within 1 degree
        return Math.abs(m_tx) < 1;
    }

    public boolean isAlignedY() {
        if (!m_hasValidTarget) return false;

        // Consider aligned if within 1 degree (adjust tolerance as needed)
        return Math.abs(m_ty) < 1;
    }
    
    public boolean hasValidTarget() {
        return m_hasValidTarget;
    }

    @Override
    public void periodic() {
        updateTargetData();

        SmartDashboard.putBoolean("Vision: Has Target", m_hasValidTarget);
        SmartDashboard.putNumber("Vision: TX", m_tx);
        SmartDashboard.putNumber("Vision: TY", m_ty);
        SmartDashboard.putNumber("Vision: X Speed", calculateXSpeed());
        SmartDashboard.putNumber("Vision: Y Speed", calculateYSpeed());
        SmartDashboard.putBoolean("Vision: Is Aligned X", isAlignedX());
        SmartDashboard.putBoolean("Vision: Is Aligned Y", isAlignedY());
    }
}