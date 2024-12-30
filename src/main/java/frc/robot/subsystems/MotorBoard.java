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
  // subsystem instance
  private static MotorBoard m_MotorBoard = null;

  // mottor instantiatinons
  private CANSparkMax neo1 = new CANSparkMax(Constants.neo1ID, MotorType.kBrushless);
  private CANSparkMax neo2 = new CANSparkMax(Constants.neo2ID, MotorType.kBrushless);
  private TalonSRX cim1 = new TalonSRX(Constants.cim1ID);
  private TalonSRX cim2 = new TalonSRX(Constants.cim2ID);

  // talon srx's don't have a good method to keep track of their current output percent so we gotta keep track of that ourselves
  private double realCim1Output = 0.0;
  private double realCim2Output = 0.0;

  // keeps track of the target speed we want every motor to be
  private double speedArray[] = new double[Constants.numberOfMotors];

  // keeps track if any motor is inversed
  private boolean inverseArray[] = new boolean[Constants.numberOfMotors];

  // returns the single instance of the subsystem
  public static MotorBoard getInstance(){
    if (m_MotorBoard == null){
      m_MotorBoard = new MotorBoard();
    }
    return m_MotorBoard;
  }

  // returns target speed for a specific motor based on index
  public double getXSpeed(int x){
    return speedArray[x];
  }

  // sets a target speed for a specific motor based on index
  public void setSpecificSpeed(int index ,double speed){
    speedArray[index] = speed;
  }

  // pretty self explanitory just read the name of the method
  public void setAllSpeed(double speed){
    for (int i = 0; i < speedArray.length; i++)
      speedArray[i] = speed;
  }

  // sets a specific boolean inverse array value based on the motors index
  public void setInverse(int index, boolean set){
    inverseArray[index] = set;
  }

  // this method runs periodically to smooth the transition between desired motor speeds
  // by only increasing real motor speed by a set constant called maxNeoPercentChange
  private void smoothTransition(CANSparkMax neo, int neoIndex){
    if (!inverseArray[neoIndex]){
      if (Math.abs(speedArray[neoIndex] - neo.get()) < Constants.maxNeoPercentChange)
        neo.set(speedArray[neoIndex]);
      else if (neo.get() > speedArray[neoIndex])
        neo.set(neo.get() - Constants.maxNeoPercentChange);
      else if (neo.get() < speedArray[neoIndex])
        neo.set(neo.get() + Constants.maxNeoPercentChange);
    }
    else{
      if (Math.abs(-speedArray[neoIndex] - neo.get()) < Constants.maxNeoPercentChange)
        neo.set(-speedArray[neoIndex]);
      else if (neo.get() > -speedArray[neoIndex])
        neo.set(neo.get() - Constants.maxNeoPercentChange);
      else if (neo.get() < -speedArray[neoIndex])
        neo.set(neo.get() + Constants.maxNeoPercentChange);
    }
  }
  // this method runs periodically to smooth the transition between desired motor speeds
  // by only increasing real motor speed by a set constant called maxCimPercentChange
  private double smoothTransition(TalonSRX cim, int cimIndex, double realCimOutput){
    if (!inverseArray[cimIndex]){
      if (Math.abs(speedArray[cimIndex] - realCimOutput) < Constants.maxCimPercentChange)
        realCimOutput = speedArray[cimIndex];
      else if (realCimOutput > speedArray[cimIndex])
        realCimOutput -= Constants.maxCimPercentChange;
      else if (realCimOutput < speedArray[cimIndex])
        realCimOutput += Constants.maxCimPercentChange;
    }
    else{
      if (Math.abs(-speedArray[cimIndex] - realCimOutput) < Constants.maxCimPercentChange)
        realCimOutput = -speedArray[cimIndex];
      else if (realCimOutput > -speedArray[cimIndex])
        realCimOutput -= Constants.maxCimPercentChange;
      else if (realCimOutput < -speedArray[cimIndex])
        realCimOutput += Constants.maxCimPercentChange;
    }
    cim.set(TalonSRXControlMode.PercentOutput, realCimOutput);
    return realCimOutput;
  }

  @Override
  public void periodic() {
    // smoothes transition between speeds by only increasing motor speed my the maximum speed percent change constant
    smoothTransition(neo1, Constants.neo1Index);
    smoothTransition(neo2, Constants.neo2Index);

    // talon srx's don't want to be good boys and report correct output percentages so we got to keep track of that ourselves
    realCim1Output = smoothTransition(cim1, Constants.cim1Index, realCim1Output);
    realCim2Output = smoothTransition(cim2, Constants.cim2Index, realCim2Output);
  }
}
