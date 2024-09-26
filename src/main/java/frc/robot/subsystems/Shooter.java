// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static final double DEADZONE = 0.05;
  private TalonSRX m_ShooterMotorLeft;
  private TalonSRX m_ShooterMotorRight;

  private double realLeftSpeed = Constants.ShooterConstants.shooterLeftSpeedSpeaker;
  private double realRightSpeed = Constants.ShooterConstants.shooterRightSpeedSpeaker;
  static Shooter instance = null;
  /** Creates a new Shooter. */
  public Shooter() {
    m_ShooterMotorLeft = new TalonSRX(Constants.ShooterConstants.shooterLeftID);
    m_ShooterMotorRight = new TalonSRX(Constants.ShooterConstants.shooterRightID);
    // Preferences.initDouble(Constants.ShooterConstants.leftSpeedKey, realLeftSpeed);
    Preferences.initDouble(Constants.ShooterConstants.rightSpeedKey, realRightSpeed);
  }

  // Retrieve instance of shooter
  public static Shooter getInstance(){
    if(instance == null) instance = new Shooter();
    return instance;
  }

  // move shooter motors
  public void shoot(double rotateLeft, double rotateRight){
    if(Math.abs(rotateLeft) < DEADZONE) rotateLeft = 0;
    if(Math.abs(rotateRight) < DEADZONE) rotateRight = 0;
    moveLeftMotor(rotateLeft);
    moveRightMotor(rotateRight);
  }

  // move left motor
  public void moveLeftMotor(double rotateLeft){
    if(Math.abs(rotateLeft) < DEADZONE) rotateLeft = 0;
    m_ShooterMotorLeft.set(TalonSRXControlMode.PercentOutput, rotateLeft);
  }

  // move right motor
  public void moveRightMotor(double rotateRight){
    if(Math.abs(rotateRight) < DEADZONE) rotateRight = 0;
    m_ShooterMotorRight.set(TalonSRXControlMode.PercentOutput, rotateRight);
  }

  public double getCurrent() {
    return (m_ShooterMotorLeft.getSupplyCurrent() + m_ShooterMotorRight.getSupplyCurrent()) / 2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current left", m_ShooterMotorLeft.getSupplyCurrent());
    SmartDashboard.putNumber("current right", m_ShooterMotorRight.getSupplyCurrent());


    Constants.ShooterConstants.shooterLeftSpeedSpeaker = Preferences.getDouble(Constants.ShooterConstants.leftSpeedKey, realLeftSpeed);
    Constants.ShooterConstants.shooterRightSpeedSpeaker = Preferences.getDouble(Constants.ShooterConstants.rightSpeedKey, realRightSpeed);
  


    // SmartDashboard.putNumber("Speed Left Speaker", Constants.ShooterConstants.shooterLeftSpeedSpeaker);
    // SmartDashboard.putNumber("Speed Right Speaker", Constants.ShooterConstants.shooterRightSpeedSpeaker);
    // SmartDashboard.putNumber("Speed Left Amp", Constants.ShooterConstants.shooterLeftSpeedAmp);
    // SmartDashboard.putNumber("Speed Right Amp", Constants.ShooterConstants.shooterRightSpeedAmp);
    // SmartDashboard.putNumber("Outtake speed", Constants.IntakeConstants.rollerSpeedOuttake);

    // Constants.ShooterConstants.shooterLeftSpeedSpeaker = SmartDashboard.getNumber("Speed Left Speaker", Constants.ShooterConstants.shooterLeftSpeedSpeaker);
    // Constants.ShooterConstants.shooterRightSpeedSpeaker = SmartDashboard.getNumber("Speed Right Speaker", Constants.ShooterConstants.shooterRightSpeedSpeaker);
    // Constants.ShooterConstants.shooterLeftSpeedAmp = SmartDashboard.getNumber("Speed Left Amp", Constants.ShooterConstants.shooterLeftSpeedAmp);
    // Constants.ShooterConstants.shooterRightSpeedAmp = SmartDashboard.getNumber("Speed Right Amp", Constants.ShooterConstants.shooterRightSpeedAmp);
    // Constants.IntakeConstants.rollerSpeedOuttake = SmartDashboard.getNumber("Outtake speed", Constants.IntakeConstants.rollerSpeedOuttake);
    // This method will be called once per scheduler run
  }
}