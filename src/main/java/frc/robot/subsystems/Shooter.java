// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of  
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.Preferences;  
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
import edu.wpi.first.wpilibj2.command.SubsystemBase;  
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static final double DEADZONE = 0.05;
  private CANSparkMax m_ShooterMotorLeft;  
  private CANSparkMax m_ShooterMotorRight;  

  private double realLeftSpeed = Constants.ShooterConstants.shooterLeftSpeedSpeaker;  
  private double realRightSpeed = Constants.ShooterConstants.shooterRightSpeedSpeaker;  
  static Shooter instance = null;

  /** Creates a new Shooter. */  
  public Shooter() {
    m_ShooterMotorLeft = new CANSparkMax(Constants.ShooterConstants.shooterLeftID, MotorType.kBrushless);  
    m_ShooterMotorRight = new CANSparkMax(Constants.ShooterConstants.shooterRightID, MotorType.kBrushless);  
    Preferences.initDouble(Constants.ShooterConstants.leftSpeedKey, realLeftSpeed);
    Preferences.initDouble(Constants.ShooterConstants.rightSpeedKey, realRightSpeed);
  }

  // Retrieve instance of shooter  
  public static Shooter getInstance(){
    if(instance == null) instance = new Shooter();  
    return instance;  
  }

  // Move shooter motors  
  public void shoot(double rotateLeft, double rotateRight){
    if(Math.abs(rotateLeft) < DEADZONE) rotateLeft = 0;  
    if(Math.abs(rotateRight) < DEADZONE) rotateRight = 0;  
    moveLeftMotor(rotateLeft);  
    moveRightMotor(rotateRight);  
  }

  // Move left motor  
  public void moveLeftMotor(double rotateLeft){
    if(Math.abs(rotateLeft) < DEADZONE) rotateLeft = 0;  
    m_ShooterMotorLeft.set(rotateLeft);  
  }

  // Move right motor  
  public void moveRightMotor(double rotateRight){
    if(Math.abs(rotateRight) < DEADZONE) rotateRight = 0;  
    m_ShooterMotorRight.set(rotateRight);  
  }

  public double getCurrent() {
    return (m_ShooterMotorLeft.getOutputCurrent() + m_ShooterMotorRight.getOutputCurrent()) / 2;  
  }

  @Override  
  public void periodic() {
    SmartDashboard.putNumber("current left", m_ShooterMotorLeft.getOutputCurrent());  
    SmartDashboard.putNumber("current right", m_ShooterMotorRight.getOutputCurrent());  

    Constants.ShooterConstants.shooterLeftSpeedSpeaker = Preferences.getDouble(Constants.ShooterConstants.leftSpeedKey, realLeftSpeed);  
    Constants.ShooterConstants.shooterRightSpeedSpeaker = Preferences.getDouble(Constants.ShooterConstants.rightSpeedKey, realRightSpeed);  
  }
}