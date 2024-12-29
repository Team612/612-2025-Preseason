package frc.robot;
public final class Constants {
  public static final int numberOfMotors = 4;

  // the CAN ID for each motor
  public static final int neo1ID = 1;
  public static final int neo2ID = 2;
  public static final int cim1ID = 1;
  public static final int cim2ID = 4;

  // these are the corresponding indexes for each motor in the speed array
  public static final int neo1Index = 0;
  public static final int neo2Index = 1;
  public static final int cim1Index = 2;
  public static final int cim2Index = 3;
  
  // this is the maximum amount of percent speed the motors can change during a tick
  // this is to prevent motors from jolting around too much and potentially breaking things
  public static double maxNeoPercentChange = 0.02;
  public static double maxCimPercentChange = 0.08;
}
