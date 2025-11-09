package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AutoAlign extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final boolean slowMode;
    private final double TARGET_DISTANCE = 0.305; // 1 foot in meters

    public AutoAlign(SwerveSubsystem swerveSubsystem, boolean slowMode) {
        this.m_SwerveSubsystem = swerveSubsystem;
        this.slowMode = slowMode;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // Check if we see a tag
        if (!LimelightHelpers.getTV("limelight")) {
            m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), slowMode);
            return;
        }

        // Get target pose in robot space
        double[] targetPoseRobotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        
        if (targetPoseRobotSpace == null || targetPoseRobotSpace.length < 6) {
            m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), slowMode);
            return;
        }

        // Target position relative to robot (x = forward, y = left)
        double targetX = targetPoseRobotSpace[0]; 
        double targetY = targetPoseRobotSpace[1];
        
        // Calculate distance to target
        double distanceToTarget = Math.sqrt(targetX * targetX + targetY * targetY);
        
        // Calculate desired position (stop TARGET_DISTANCE away)
        double ratio = (distanceToTarget - TARGET_DISTANCE) / distanceToTarget;
        double desiredX = targetX * ratio;
        double desiredY = targetY * ratio;
        
        // Convert robot-relative desired position to field coordinates
        Pose2d currentPose = m_SwerveSubsystem.getPose();
        double fieldX = currentPose.getX() + 
                       (desiredX * Math.cos(currentPose.getRotation().getRadians()) - 
                        desiredY * Math.sin(currentPose.getRotation().getRadians()));
        double fieldY = currentPose.getY() + 
                       (desiredX * Math.sin(currentPose.getRotation().getRadians()) + 
                        desiredY * Math.cos(currentPose.getRotation().getRadians()));
        
        // Calculate angle to face the tag
        double angleToTarget = Math.atan2(targetY, targetX);
        double desiredHeading = currentPose.getRotation().getRadians() + angleToTarget;
        
        // Create target pose in field coordinates
        Pose2d targetPose = new Pose2d(
            new Translation2d(fieldX, fieldY),
            new Rotation2d(desiredHeading)
        );
        
        // Use your autoAlign method
        m_SwerveSubsystem.autoAlign(targetPose, slowMode);
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), slowMode);
    }

    @Override
    public boolean isFinished() {
        if (!LimelightHelpers.getTV("limelight")) {
            return true;
        }

        double[] targetPoseRobotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        if (targetPoseRobotSpace == null || targetPoseRobotSpace.length < 6) {
            return true;
        }

        double targetX = targetPoseRobotSpace[0];
        double targetY = targetPoseRobotSpace[1];
        double distanceToTarget = Math.sqrt(targetX * targetX + targetY * targetY);
        double distanceError = Math.abs(distanceToTarget - TARGET_DISTANCE);
        
        double angleToTarget = Math.atan2(targetY, targetX);
        
        // Finish when close enough (5cm and 5 degrees)
        return distanceError < 0.05 && Math.abs(angleToTarget) < Math.toRadians(5);
    }
}