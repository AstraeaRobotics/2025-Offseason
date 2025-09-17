// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.units.measure.Velocity;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.Slot1Configs;
// import com.ctre.phoenix6.configs.Slot2Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.CoastOut;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import frc.robot.Constants.ClimbConstants.ClimbStates;

// public class ClimbSubsystem extends SubsystemBase {

//   /** Creates a new ClimbSubsystem. */
//   private TalonFX m_climbMotor;
  
//   Slot0Configs m_climbConfigs;
//   PositionVoltage m_climbPosition;
//   VelocityVoltage m_climbVelocity;
//   // VelocityVoltage m_climbVelocity;
//   ClimbStates m_climbState;
//   double desiredSetpoint;  
//   NeutralOut brake;

//   public ClimbSubsystem() {
//     m_climbMotor = new TalonFX(7);
//     m_climbConfigs = new Slot0Configs();

//     configureMotors();

//     m_climbVelocity = new VelocityVoltage(0).withSlot(0);
//     m_climbPosition = new PositionVoltage(0).withSlot(0);
//     brake = new NeutralOut();
    
//     m_climbState = ClimbStates.kTop;
//     desiredSetpoint = m_climbState.getClimbSetpoint();

//   }
  
//   public void configureMotors() {
//     m_climbConfigs.kS = 0.13;
//     m_climbConfigs.kV = 0.1;
//     m_climbConfigs.kP = 0.25;
//     m_climbConfigs.kI = 0.0;
//     m_climbConfigs.kD = 0.0;

//     m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
//     m_climbMotor.getConfigurator().apply(m_climbConfigs);

//     // current limit

//     CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

//     currentLimits.SupplyCurrentLimit = 70.0; 
//     currentLimits.SupplyCurrentLimitEnable = true;

//     currentLimits.StatorCurrentLimit = 80.0;
//     currentLimits.StatorCurrentLimitEnable = true;

//     m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
//     m_climbMotor.getConfigurator().apply(m_climbConfigs);
//     m_climbMotor.getConfigurator().apply(currentLimits);

//     m_climbMotor.setPosition(0);
//   }

//   public void spinClimbMotorPosition(double position) {
//     m_climbMotor.setControl(m_climbPosition.withPosition(position));
//   }
//   public void spinClimbMotorVelocity(double velocity) {
//     m_climbMotor.setControl(m_climbVelocity.withVelocity(velocity));
//   }
//   public double getClimbPosition() {
//     return m_climbMotor.getPosition().getValueAsDouble();
//   }
//   public double getClimbError() {
//     return m_climbMotor.getClosedLoopError().getValueAsDouble();
//   }

//   public double getClimbVelocity() {
//     return m_climbMotor.getVelocity().getValueAsDouble();
// }

//   public void setPivotVoltage(double voltage) {
//     m_climbMotor.setVoltage(voltage);
//   }

//   public void setClimbState (ClimbStates tempState) {
//     m_climbState = tempState;
//     desiredSetpoint = m_climbState.getClimbSetpoint();
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Climb Position", getClimbPosition());
//     SmartDashboard.putNumber("Climb Velocity", getClimbVelocity());
//     spinClimbMotorPosition(desiredSetpoint);
//   }
// }

