// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import com.revrobotics.SparkLimitSwitch;

public class Intake extends SubsystemBase {
  private static final double DEADZONE = 0.05;
  private CANSparkMax m_IntakePivotMotor;
  private TalonSRX m_IntakeRollerMotor;
  private AnalogInput IRSensor = new AnalogInput(Constants.IntakeConstants.IRport);
  static Intake instance = null;
  /** Creates a new Intake. */
  public Intake() {
    m_IntakePivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    m_IntakeRollerMotor = new TalonSRX(Constants.IntakeConstants.rollerID);
    m_IntakePivotMotor.setIdleMode(IdleMode.kBrake);
  }

  // create instance of intake
  public static Intake getInstance(){
    if(instance == null){
      instance = new Intake();
    }
    return instance;
  }
  
  // return limit switch states
  public boolean getIntakeLimitStateForward(){
    return m_IntakePivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
  }

  // return limit switch states
  public boolean getIntakeLimitStateReverse(){
    return m_IntakePivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
  }



  // move intake pivot
  public void rotateIntake(double rotate){
    if(Math.abs(rotate) < DEADZONE) rotate = 0;
    m_IntakePivotMotor.set(rotate);
  }

  // move intake rollers
  public void moveRollers(double rotate){
    if(Math.abs(rotate) < DEADZONE) rotate = 0;
    m_IntakeRollerMotor.set(TalonSRXControlMode.PercentOutput, rotate);
  }

  public double getIRSensor(){
    return IRSensor.getVoltage();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("limit forward", getIntakeLimitStateForward());
    // SmartDashboard.putBoolean("limit reverse", getIntakeLimitStateReverse());
    // SmartDashboard.putNumber("IR Sensor", getIRSensor());
    // This method will be called once per scheduler run
  }
}