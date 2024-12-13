// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static final double DEADZONE = 0.05;
  private final CANSparkMax m_ShooterMotorLeft;
  private final CANSparkMax m_ShooterMotorRight;
  static Shooter instance = null;
  /** Creates a new Shooter. */
  public Shooter() {
    m_ShooterMotorLeft = new CANSparkMax(Constants.ShooterConstants.shooterLeftID, MotorType.kBrushless);
    m_ShooterMotorRight = new CANSparkMax(Constants.ShooterConstants.shooterRightID, MotorType.kBrushless);
  }

  // Retrieve instance of shooter
  public static Shooter getInstance(){
    if(instance == null) instance = new Shooter();
    return instance;
  }

  // move shooter motors
  public void shoot(double rotateLeft, double rotateRight){
    // if(rotateLeft < DEADZONE) rotateLeft = 0;
    // if(rotateRight < DEADZONE) rotateRight = 0;
    moveLeftMotor(rotateLeft);
    moveRightMotor(rotateRight);
  }

  // move left motor
  public void moveLeftMotor(double rotateLeft){
    m_ShooterMotorLeft.set(-1*rotateLeft);
  }

  // move right motor
  public void moveRightMotor(double rotateRight){
    m_ShooterMotorRight.set(rotateRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}