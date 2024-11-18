// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.ArrayList;
import com.revrobotics.CANSparkMax;

public class CANSparkMaxSubsystem extends SubsystemBase {
  private static CANSparkMaxSubsystem cansparkmax = null;
  private CANSparkMax[] motorArray;
  public CANSparkMax getXMotor(int x){
    return motorArray[x];
  }

  private ArrayList <CANSparkMax> availableMotors  = new ArrayList<>();

  public static CANSparkMaxSubsystem getInstance(){
    if (cansparkmax == null){
      cansparkmax = new CANSparkMaxSubsystem();
    }
    return cansparkmax;
  }

  // set motor IDS in order
  public CANSparkMaxSubsystem() {
    motorArray = new CANSparkMax[4];
    for (int i = 0; i < motorArray.length; i++){
      availableMotors.add(new CANSparkMax(i+1, MotorType.kBrushless));
      motorArray[i] = availableMotors.get(i);
    }
  }
  
  // manully set initial motor IDS
  /*
  public CANSparkMaxSubsystem() {
    availableMotors.add(new CANSparkMax(1, MotorType.kBrushless));
    motorArray[0] = availableMotors.get(availableMotors.size()-1);
    availableMotors.add(new CANSparkMax(2, MotorType.kBrushless));
    motorArray[1] = availableMotors.get(availableMotors.size()-1);
    availableMotors.add(new CANSparkMax(3, MotorType.kBrushless));
    motorArray[2] = availableMotors.get(availableMotors.size()-1);
    availableMotors.add(new CANSparkMax(4, MotorType.kBrushless));
    motorArray[3] = availableMotors.get(availableMotors.size()-1);
  }
  */

  public void setAllSpeed(double speed){
    for (int i = 0; i < motorArray.length; i++){
      motorArray[i].set(speed);
    }
  }

  public void setMotorsBasedOnIndex(int index,double speed){
    motorArray[index].set(speed);
  }

  public void setMotorsBasedOnCan(int CANid, double speed){
    for(int i = 0; i < motorArray.length; i++){
      if (motorArray[i].getDeviceId() == CANid)
        motorArray[i].set(speed);
    }
  }

  public void setID(int motorIndex, int newID){
    for (int i = 0; i < availableMotors.size(); i++){
      if (availableMotors.get(i).getDeviceId() == newID){
        motorArray[motorIndex] = availableMotors.get(i);
        return;
      }
    }
    availableMotors.add(new CANSparkMax(newID, MotorType.kBrushless));
    motorArray[motorIndex] = availableMotors.get(availableMotors.size()-1);
  }

  @Override
  public void periodic() {
  }
}
