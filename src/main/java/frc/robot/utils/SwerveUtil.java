package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DrivebaseModuleConstants;

public class SwerveUtil {
    public static double[] optimizeModule(double currentAngle, double desiredAngle, double speed) {
        double[] optimizedModule = new double[2];
        double angleDifference = desiredAngle - currentAngle;
        optimizedModule[0] = desiredAngle;
        optimizedModule[1] = -speed;
        
        // Optimizes the desired angle so the module never turns more than 90 degrees
        if(Math.abs(angleDifference) > 90 && Math.abs(angleDifference) < 270) {
            optimizedModule[0] = (desiredAngle + 180)  % 360;
            optimizedModule[1] = speed;
        }

        return optimizedModule;
    }

    public static double getModuleVoltage(double speedMPS, boolean slowMode) {
        return MathUtil.clamp(slowMode ? (speedMPS * DrivebaseModuleConstants.driveKV) / 1.5 : (speedMPS * DrivebaseModuleConstants.driveKV), -DrivebaseModuleConstants.kMaxDriveVoltage, DrivebaseModuleConstants.kMaxDriveVoltage);
    }

    public static ChassisSpeeds driveInputToChassisSpeeds(double driveX, double driveY, double rotation, double heading) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(-driveY, -driveX, -rotation, Rotation2d.fromDegrees(-heading));
    }

    public static ChassisSpeeds autoInputToChassisSpeeds(double driveX, double driveY, double rotation, double heading) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(-driveY, -driveX, -rotation, Rotation2d.fromDegrees(heading));
    }

    public static double driftCorrection(double ogYaw, double currentYaw) {
        return ((currentYaw) / (180)) * Math.PI * 0.1;
    }
}