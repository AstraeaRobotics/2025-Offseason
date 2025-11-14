// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralStates;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  SparkMax coralLeftIntakeMotor;
  SparkMax coralRightIntakeMotor;
  SparkMax coralPivotMotor;

  AbsoluteEncoder coralPivotEncoder;
  RelativeEncoder coralLeftIntakeEncoder;
  RelativeEncoder coralRightIntakeEncoder;

  CoralStates m_coralState;
  double coralSetpoint;

  ProfiledPIDController m_coralPidController;
  
  ArmFeedforward m_coralPivotFeedforward;
  
  public CoralSubsystem() {
    coralLeftIntakeMotor = new SparkMax(1, MotorType.kBrushless); 
    coralRightIntakeMotor = new SparkMax(2, MotorType.kBrushless); 
    coralPivotMotor = new SparkMax(3, MotorType.kBrushless);

    coralLeftIntakeEncoder = coralLeftIntakeMotor.getEncoder();
    coralRightIntakeEncoder = coralRightIntakeMotor.getEncoder();
    coralPivotEncoder = coralPivotMotor.getAbsoluteEncoder();

    m_coralState = CoralStates.kRest;
    coralSetpoint = m_coralState.getCoralSetpoint();

    m_coralPidController = new ProfiledPIDController(CoralConstants.kP, 0, CoralConstants.kD, new TrapezoidProfile.Constraints(10.0, 12.5));

    m_coralPivotFeedforward = new ArmFeedforward(0, CoralConstants.coralPivotkG, 0, 0);
    configureMotors();
  }

  private void configureMotors(){

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig rightIntakeConfig = new SparkMaxConfig();
    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();

    pivotConfig.smartCurrentLimit(35).idleMode(IdleMode.kBrake).inverted(false);
    leftIntakeConfig.smartCurrentLimit(20).idleMode(IdleMode.kCoast);
    rightIntakeConfig.smartCurrentLimit(20).inverted(true).idleMode(IdleMode.kCoast);

    m_coralPidController.enableContinuousInput(0,1);

    coralLeftIntakeMotor.configure(leftIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralRightIntakeMotor.configure(rightIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralPivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public void setCoralPivotMotor(double speed){
    coralPivotMotor.set(speed);
  }

  public void setLeftVoltage(double voltage){
    coralLeftIntakeMotor.setVoltage(voltage);         
  }

  public void setRightVoltage(double voltage){
    coralRightIntakeMotor.setVoltage(voltage);         
  }

  public void setIntakeVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setPivotVoltage(double voltage){
    coralPivotMotor.setVoltage(voltage);         
  }

  public double getPivotEncoder(){
    return (coralPivotEncoder.getPosition());
  }

  public void setState(CoralStates tempState){
    m_coralState = tempState;
    coralSetpoint = m_coralState.getCoralSetpoint();
  }
  public CoralStates getCoralStates(){
    return m_coralState;
  }

  public double getPivotPID(){
    return m_coralPidController.calculate(getPivotEncoder(), coralSetpoint);
  }

  public double getEncoderFF() {
    return 2 * Math.PI * getPivotEncoder();
  }

  public double getPivotOutput() {
    return (MathUtil.clamp(-getPivotPID() + m_coralPivotFeedforward.calculate(coralSetpoint * 2 * Math.PI, 0), -6, 6));
  }

  public void setPivotPID(){
    coralPivotMotor.set(MathUtil.clamp(getPivotPID(),-0.5,0.5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral encoder position", getPivotEncoder());
    SmartDashboard.putNumber("coral setpoint", coralSetpoint);
    coralPivotMotor.setVoltage(getPivotOutput());
  }
}
