// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

public class MotorBoard extends SubsystemBase {
  
  // instantiates all of the datafields of the motor board class
  private static MotorBoard m_MotorBoard = null;
  private CANSparkMax neo1 = new CANSparkMax(Constants.neo1ID, MotorType.kBrushless);
  private CANSparkMax neo2 = new CANSparkMax(Constants.neo2ID, MotorType.kBrushless);
  private TalonSRX cim1 = new TalonSRX(Constants.cim1ID);
  private TalonSRX cim2 = new TalonSRX(Constants.cim2ID);
  private double realCim1Output = 0.0;
  private double realCim2Output = 0.0;
  private double speedArray[] = new double[Constants.numberOfMotors];
  private boolean inverseArray[] = new boolean[Constants.numberOfMotors];
  public static MotorBoard getInstance(){
    if (m_MotorBoard == null){
      m_MotorBoard = new MotorBoard();
    }
    return m_MotorBoard;
  }
  public MotorBoard() {
  }

  public double getXSpeed(int x){
    return speedArray[x];
  }

  public void setAllSpeed(double speed){
    for (int i = 0; i < speedArray.length; i++)
      speedArray[i] = speed;
  }

  public void setInverse(int index, boolean set){
    inverseArray[index] = set;
  }

  public void setSpecificSpeed(int index ,double speed){
    speedArray[index] = speed;
  }

  @Override
  public void periodic() {
    // these four modules of code make the motors smoothly transition to the speed given by the speed array
    if (!inverseArray[Constants.neo1Index]){
      if (Math.abs(speedArray[Constants.neo1Index] - neo1.get()) < Constants.maxNeoPercentChange)
        neo1.set(speedArray[Constants.neo1Index]);
      else if (neo1.get() > speedArray[Constants.neo1Index])
        neo1.set(neo1.get() - Constants.maxNeoPercentChange);
      else if (neo1.get() < speedArray[Constants.neo1Index])
        neo1.set(neo1.get() + Constants.maxNeoPercentChange);
    }
    else{
      if (Math.abs(-speedArray[Constants.neo1Index] - neo1.get()) < Constants.maxNeoPercentChange)
        neo1.set(-speedArray[Constants.neo1Index]);
      else if (neo1.get() > -speedArray[Constants.neo1Index])
        neo1.set(neo1.get() - Constants.maxNeoPercentChange);
      else if (neo1.get() < -speedArray[Constants.neo1Index])
        neo1.set(neo1.get() + Constants.maxNeoPercentChange);
    }
    
    if (!inverseArray[Constants.neo2Index]){
      if (Math.abs(speedArray[Constants.neo2Index] - neo2.get()) < Constants.maxNeoPercentChange)
        neo2.set(speedArray[Constants.neo2Index]);
      else if (neo2.get() > speedArray[Constants.neo2Index])
        neo2.set(neo2.get() - Constants.maxNeoPercentChange);
      else if (neo2.get() < speedArray[Constants.neo2Index])
        neo2.set(neo2.get() + Constants.maxNeoPercentChange);
    }
    else{
      if (Math.abs(-speedArray[Constants.neo2Index] - neo2.get()) < Constants.maxNeoPercentChange)
        neo2.set(-speedArray[Constants.neo2Index]);
      else if (neo2.get() > -speedArray[Constants.neo2Index])
        neo2.set(neo2.get() - Constants.maxNeoPercentChange);
      else if (neo2.get() < -speedArray[Constants.neo2Index])
        neo2.set(neo2.get() + Constants.maxNeoPercentChange);
    }
    

    if (!inverseArray[Constants.cim1Index]){
      if (Math.abs(speedArray[Constants.cim1Index] - realCim1Output) < Constants.maxCimPercentChange)
        realCim1Output = speedArray[Constants.cim1Index];
      else if (realCim1Output > speedArray[Constants.cim1Index])
        realCim1Output -= Constants.maxCimPercentChange;
      else if (realCim1Output < speedArray[Constants.cim1Index])
        realCim1Output += Constants.maxCimPercentChange;
    }
    else{
      if (Math.abs(-speedArray[Constants.cim1Index] - realCim1Output) < Constants.maxCimPercentChange)
        realCim1Output = -speedArray[Constants.cim1Index];
      else if (realCim1Output > -speedArray[Constants.cim1Index])
        realCim1Output -= Constants.maxCimPercentChange;
      else if (realCim1Output < -speedArray[Constants.cim1Index])
        realCim1Output += Constants.maxCimPercentChange;
    }
    
    if (!inverseArray[Constants.cim2Index]){
      if (Math.abs(speedArray[Constants.cim2Index] - realCim2Output) < Constants.maxCimPercentChange)
        realCim2Output = speedArray[Constants.cim2Index];
      else if (realCim2Output > speedArray[Constants.cim2Index])
        realCim2Output -= Constants.maxCimPercentChange;
      else if (realCim2Output < speedArray[Constants.cim2Index])
        realCim2Output += Constants.maxCimPercentChange;
    }
    else{
      if (Math.abs(-speedArray[Constants.cim2Index] - realCim2Output) < Constants.maxCimPercentChange)
        realCim2Output = -speedArray[Constants.cim2Index];
      else if (realCim2Output > -speedArray[Constants.cim2Index])
        realCim2Output -= Constants.maxCimPercentChange;
      else if (realCim2Output < -speedArray[Constants.cim2Index])
        realCim2Output += Constants.maxCimPercentChange;
    }

    cim1.set(TalonSRXControlMode.PercentOutput, realCim1Output);
    cim2.set(TalonSRXControlMode.PercentOutput, realCim2Output);
  }
}
