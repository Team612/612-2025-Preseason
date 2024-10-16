// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rollers extends SubsystemBase {

  private static final double DEADZONE = 0.05;
  private TalonSRX m_IntakeRollerMotor;
  private double realRollerSpeed = Constants.IntakeConstants.rollerSpeedIntake;  
  
  private AnalogInput IRSensor = new AnalogInput(Constants.IntakeConstants.IRport);
  static Rollers instance = null;

  /** Creates a new Rollers. */
  public Rollers() {
        m_IntakeRollerMotor = new TalonSRX(Constants.IntakeConstants.rollerID);
  }

  public static Rollers getInstance(){
    if(instance == null){
      instance = new Rollers();
    }
    return instance;
  }

  public void moveRollers(double rotate){
    if(Math.abs(rotate) < DEADZONE) rotate = 0;
    m_IntakeRollerMotor.set(TalonSRXControlMode.PercentOutput, rotate);
  }

  public double getIRSensor(){
    return IRSensor.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(Constants.IntakeConstants.rollerInKey, m_IntakeRollerMotor.getSupplyCurrent());  

    Constants.ShooterConstants.shooterRightSpeedSpeaker = Preferences.getDouble(Constants.IntakeConstants.rollerInKey, realRollerSpeed); 
  }
}
