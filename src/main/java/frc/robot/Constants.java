// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class CoralConstants {
    public static final double kP = 13.0;
    public static final double kI = 0.0;
    public static final double kD = 1.0;

    //feed forward constants 
    public static final double coralIntakekS = 0.0;
    public static final double coralIntakekV = 0.0;
    public static final double coralIntakekA = 0.0;

    public static final double coralPivotkS = 0.0;
    public static final double coralPivotkG = 0.59; 
    public static final double coralPivotkV = 0.0;
    public static final double coralPivotkA = 0.0;

    public enum CoralStates{
      kRest(0.823),
      kL1(0.97),
      kL2(0),
      kL3(0.96),
      kSource(0.915);
      private double coralSetpoint;

      private CoralStates(double coralSetpoint){
        this.coralSetpoint = coralSetpoint;
      }

      public double getCoralSetpoint(){
        return coralSetpoint;
      }
    }
  }

  public static class ElevatorConstants{
    public static final double kEncoderConversionFactor = 2*Math.PI;
    public static final double kP = 1.4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0.33;
    public static final double kV = 0;
    public static final double kA = 0;
    
    public enum ElevatorStates{  
      kRest(0),
      kSource(21), 
      kProcessor(23),
      kCL1(3.5),
      kCL2(21.5),
      kCL3(43),
      kAl1(10),
      kAL2(37),
      kAl3(43);


      private double elevatorSetPoint;

      private ElevatorStates(double elevatorSetPoint) {
        this.elevatorSetPoint = elevatorSetPoint;
      }
      
      public double getElevatorSetPoint(){
        return elevatorSetPoint;
      }
    }
  }

  public static class DrivebaseModuleConstants {
        public static final double kDriveGearRatio = 3.56;
        public static final double kWheelDiameter = Units.inchesToMeters(3);
        public static final double kMaxDriveVoltage = 6.0;

        public static final int kTurnEncoderPositionFactor = 360;
        public static final int kTurnEncoderVelocityFactor = 60;

        public static final double kDriveEncoderPositionFactor = (1 / kDriveGearRatio) * 2 * Math.PI * (kWheelDiameter / 2);
        public static final double kDriveEncoderVelocityFactor = 1/(60 * kDriveGearRatio);

        public static final double turnKP = 0.004;
        public static final double turnKI = 0;
        public static final double turnKD = 0;

        public static final double driveKV = 6.5; 
        public static final double driveKS = 0.25;
  }

  public static class DrivebaseConstants {
    public static final double kWheelBase = Units.inchesToMeters(26.125);
    public static final double kTrackWidth = Units.inchesToMeters(23.75);

    public static final double kAutoSpeedMultiplier = 0.8;

    public static final double kRobotCentricVel = 0.1;
  }

  public static class VisionConstants {
    public static final double kXP = 0.0098;
    public static final double kXI = 0.0;
    public static final double kXD = 0.0;
    
    public static final double kYP = 0.05;
    public static final double kYI = 0.0;
    public static final double kYD = 0.0;

    public static final double kRotP = 0.02; 
    public static final double kRotI = 0.0;
    public static final double kRotD = 0.001;
    
    public static final double kXTolerance = 0.5; 
    public static final double kYTolerance = 0.1;
    public static final double kRotationTolerance = 5.0;

    public static final double kTargetTY = 5.5;
    
    public enum AlignmentPosition {
      CENTER(0.0),
      LEFT_EDGE(-(Units.inchesToMeters(5))), 
      RIGHT_EDGE(Units.inchesToMeters(5));  

      private final double offsetMeters;

      AlignmentPosition(double offsetMeters) {
        this.offsetMeters = offsetMeters;
      }

      public double getOffsetMeters() {
        return offsetMeters;
      }
    }
  }

  public static final class JustSomeConstants {
    public static final double poseX = 1.9;
    public static final double poseY = 1.03;
    public static final double poseRot = -30;
  }

  public static final class ClimbConstants { 

    public enum ClimbStates {
      kTop(0), 
      kGround(-45);
  
      private double climbSetpoint;
      private ClimbStates(double climbSetpoint){
        this.climbSetpoint = climbSetpoint;
      }
      public double getClimbSetpoint(){
        return climbSetpoint;
      }
    }
  }
}