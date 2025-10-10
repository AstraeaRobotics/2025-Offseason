// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


// NetworkTableInstance defaultInst = NetworkTableInstance.getDefault();


public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  private final SparkMax m_pivot;
  private final SparkMax m_intakeL;
  private final SparkMax m_intakeR;

  private final AbsoluteEncoder m_pivotEncoder;
  private final RelativeEncoder m_intakeEncoder;
  

  private final ProfiledPIDController m_pid;
  private final SimpleMotorFeedforward m_intakeFeedForward;
  private final ArmFeedforward m_pivotFeedforward;
  private AlgaeStates currState;
  public double desiredSetpoint;
  
  
  public AlgaeSubsystem() {
    m_pivot = new SparkMax(6, MotorType.kBrushless);
    m_intakeL = new SparkMax(4, MotorType.kBrushless);
    m_intakeR = new SparkMax(5, MotorType.kBrushless);
    
    //  14
    m_pid = new ProfiledPIDController(16.0, 0, 1.0, new TrapezoidProfile.Constraints(0.4, 0.8));
    m_pivotEncoder = m_pivot.getAbsoluteEncoder();
    
    m_intakeEncoder = m_intakeL.getEncoder();
    m_intakeEncoder.setPosition(0);

    configMotors();
    
    currState = AlgaeStates.kIn;
    desiredSetpoint = currState.getPivotPos();
    
    m_intakeFeedForward = new SimpleMotorFeedforward(AlgaeConstants.intakeKS, AlgaeConstants.intakeKV, AlgaeConstants.intakeKA);
    m_pivotFeedforward = new ArmFeedforward(0, AlgaeConstants.pivotKG, 0);

  }

  private void configMotors(){
    /*
     * This SparkMaxConfig is intended to be temporary, it would be better for a 
     * config object to be created in the constants class and imported into this class
     */

    SparkMaxConfig configPivot = new SparkMaxConfig();
    
    configPivot.smartCurrentLimit(35).idleMode(IdleMode.kBrake);
    // configPivot.closedLoopRampRate(10);

    m_pivot.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configIntakeL = new SparkMaxConfig();

    configIntakeL.smartCurrentLimit(20);
    configIntakeL.closedLoopRampRate(10);

    m_intakeL.configure(configIntakeL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configIntakeR = new SparkMaxConfig();

    configIntakeR.smartCurrentLimit(20);
    configIntakeR.closedLoopRampRate(10);
    configIntakeR.inverted(true);

    m_pid.enableContinuousInput(0,1);


    m_intakeR.configure(configIntakeR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPivotAngle(){
    //add conversion factor here
    return 1-m_pivotEncoder.getPosition();
  }

  public double getIntakeVelocity(){
    return m_intakeEncoder.getVelocity();
  }

  public double getPivotVelocity(){
    return m_pivotEncoder.getVelocity();
  }

  public double getPID(){
    return m_pid.calculate(getPivotAngle(), desiredSetpoint);
  }

  public void setPivotManual(double speed){
    m_pivot.set(speed);
  }

  public void setIntakeVoltageManual(double voltage){
    m_intakeL.setVoltage(voltage);
    m_intakeR.setVoltage(voltage * 1.35);
  }

  public void setPivotVoltageManual(double voltage){
    m_pivot.setVoltage(voltage);
  }

  public void setState(AlgaeStates newState){
    currState = newState;
    desiredSetpoint = currState.getPivotPos();
  }

  public void setIntake(double speed){
    m_intakeR.setVoltage(m_intakeFeedForward.calculate(speed));
    m_intakeL.setVoltage(m_intakeFeedForward.calculate(speed));
  }

  public void setPivot(){
    m_pivot.setVoltage(MathUtil.clamp(getPivotOutput(), -5, 5));
  }

  private double getPivotOutput() {
    return getPID() + m_pivotFeedforward.calculate(desiredSetpoint * 2 * Math.PI - (Math.PI / 2), 0);
  }


  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae position", getPivotAngle());
    SmartDashboard.putNumber("Algae desired setpoint", desiredSetpoint);
    SmartDashboard.putNumber("pid output", getPivotOutput());
    setPivot();
  }
}
