// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private final CANSparkMax spark_fl;
  private final CANSparkMax spark_bl; 
  private final CANSparkMax spark_fr;  
  private final CANSparkMax spark_br; 
  public static Servo leftServo;
  public static Servo rightServo;

  public Drivetrain() {
    spark_fl = new CANSparkMax(Constants.SPARK_FL, MotorType.kBrushless);
    spark_bl = new CANSparkMax(Constants.SPARK_BL, MotorType.kBrushless);
    spark_fr = new CANSparkMax(Constants.SPARK_FR, MotorType.kBrushless);
    spark_br = new CANSparkMax(Constants.SPARK_BR, MotorType.kBrushless);

    leftServo = new Servo(Constants.LEFT_SERVO_PORT);
    rightServo = new Servo(Constants.RIGHT_SERVO_PORT);
  }

  public final CANSparkMax getSparkLeft(int num) {
    if(num == 0) {
      return spark_fl;
    } 
    else {
     return spark_bl;
    }
  }

  public final CANSparkMax getSparkRight(int num){
    if(num == 0){
      return spark_fr;

    } 
    else {
     return spark_br;
    }
  }
  /** Creates a new Drivetrain. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}