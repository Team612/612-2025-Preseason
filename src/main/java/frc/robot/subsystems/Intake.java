// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
  private DigitalInput IRSensor = new DigitalInput(Constants.IntakeConstants.IRport);
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
  public boolean getIntakeLimitState(){
    return m_IntakePivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
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

  public boolean getIRSensor(){
    return IRSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}