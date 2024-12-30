package frc.robot;
public final class Constants {

  // number of motors
  public static final int numberOfMotors = 4;

  // the CAN ID for each motor
  public static final int neo1ID = 1;
  public static final int neo2ID = 2;
  public static final int cim1ID = 1;
  public static final int cim2ID = 4;

  // these are the corresponding indexes for each motor in the speed array and inverse array
  // !IMPORTANT NOTE! if you increase or decrease the amount of motors, please change every other index to fit the arrays or else it wont work 
  public static final int neo1Index = 0;
  public static final int neo2Index = 1;
  public static final int cim1Index = 2;
  public static final int cim2Index = 3;
  
  // this is the maximum amount of percent speed the motors can change during a tick
  // this is to prevent motors from jolting around too much and potentially breaking things
  // Cim motors are thoughtfully set higher because I noticed they are less jumpy and it is ok to have a higher max percent change
  public static double maxNeoPercentChange = 0.02;
  public static double maxCimPercentChange = 0.08;
}
