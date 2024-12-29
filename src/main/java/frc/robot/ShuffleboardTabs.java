package frc.robot;

import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.MotorBoard;

public class ShuffleboardTabs {
    // defines tabs
    private ShuffleboardTab MotorsTab;
    
    // defines all of the buttons on the shuffleboard
    private GenericEntry[] motorSpeeds = new GenericEntry[Constants.numberOfMotors];
    private GenericEntry[] syncButtons = new GenericEntry[Constants.numberOfMotors];
    private GenericEntry[] inverseButtons = new GenericEntry[Constants.numberOfMotors];
    private GenericEntry neoMotorMaxPercentChange;
    private GenericEntry cimMotorMaxPercentChange;
    private GenericEntry allspeedsetter;
    private GenericEntry disableButtons;
    
    // gets subsystem
    private MotorBoard m_MotorBoard = MotorBoard.getInstance();
    //initializes all the buttons with default values
    public void initButton(){
        MotorsTab = Shuffleboard.getTab("Motors Tab");
        neoMotorMaxPercentChange = MotorsTab.add("Neo Motor Max % Change", Constants.maxNeoPercentChange).withWidget(BuiltInWidgets.kTextView).withPosition(3,0).withSize(2,1).getEntry();
        cimMotorMaxPercentChange = MotorsTab.add("Cim Motor Max % Change", Constants.maxCimPercentChange).withWidget(BuiltInWidgets.kTextView).withPosition(5,0).withSize(2,1).getEntry();
        allspeedsetter = MotorsTab.add("Set all CAN instance speeds", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).withPosition(0,0).withSize(3,1).getEntry();
        disableButtons = MotorsTab.add("Disable", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(7, 0).getEntry();
        for(int i = 0; i < Constants.numberOfMotors; i++){
                motorSpeeds[i] = MotorsTab.add("Motor " + (i+1) + "Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(i*2,1).withSize(2,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                syncButtons[i] = MotorsTab.add("Sync Motor " + (i +1), false).withWidget(BuiltInWidgets.kToggleButton).withPosition(i*2, 2).withSize(2,1).getEntry();
                inverseButtons[i] = MotorsTab.add("Inverse " + (i +1), false).withWidget(BuiltInWidgets.kToggleButton).withPosition(i*2, 3).withSize(2,1).getEntry();
        }
    }

    // defines the last shuffleboard values in variables
    // because to see if the user has inputed something we just compare to see if the last has changed from the current
    private double[] lastSpeeds = new double[Constants.numberOfMotors];
    double lastallspeedsetter = 0.0;
    boolean lastDisable = false;

    // runs periodicly to check if a user has inputed something and then changes its actual value and updates shufflboard
    public void updateButtons(){
        if (neoMotorMaxPercentChange.get().getDouble() < 0.001)
            neoMotorMaxPercentChange.setDouble(0.001);
        if (neoMotorMaxPercentChange.get().getDouble() > 1)
            neoMotorMaxPercentChange.setDouble(1.0);
        if (cimMotorMaxPercentChange.get().getDouble() < 0.001)
            cimMotorMaxPercentChange.setDouble(0.001);
        if (cimMotorMaxPercentChange.get().getDouble() > 1)
            cimMotorMaxPercentChange.setDouble(1.0);
        Constants.maxNeoPercentChange = neoMotorMaxPercentChange.get().getDouble();
        Constants.maxCimPercentChange = cimMotorMaxPercentChange.get().getDouble();
        for (int i = 0; i < Constants.numberOfMotors; i++){
            m_MotorBoard.setInverse(i, inverseButtons[i].get().getBoolean());
        }
        // defines the current values in shuffleboard and adds them to variables 
        // because to see if the user has inputed something we just compare to see if the last has changed from the current
        double currallspeedsetter = allspeedsetter.get().getDouble();
        double[] currSpeeds = new double[Constants.numberOfMotors];
            for(int i = 0; i < currSpeeds.length; i++)
                currSpeeds[i] = motorSpeeds[i].get().getDouble();
        if (!disableButtons.get().getBoolean()){
            // sets the speed array
            // checks if the user actually inputed something first so it doesn't spam inputs
            if (currallspeedsetter != lastallspeedsetter)
                m_MotorBoard.setAllSpeed(currallspeedsetter);
            for (int i = 0; i < Constants.numberOfMotors; i++){
                if (currSpeeds[i] != lastSpeeds[i]){
                    m_MotorBoard.setSpecificSpeed(i, currSpeeds[i]);
                    if (syncButtons[i].get().getBoolean()){
                        for (int j = 0; j < Constants.numberOfMotors; j++){
                            if (syncButtons[j].get().getBoolean())
                                m_MotorBoard.setSpecificSpeed(j, currSpeeds[i]);
                        }
                    }
                    break;
                }
            }
            if (lastDisable){
                for (int i = 0; i < Constants.numberOfMotors; i++){
                    m_MotorBoard.setSpecificSpeed(i, currSpeeds[i]);
                }
            }

            // updates shufflboard based on real values which have been changed by now
            for (int i = 0; i < Constants.numberOfMotors-1; i++){
                if (m_MotorBoard.getXSpeed(i) != m_MotorBoard.getXSpeed(i+1)){
                    allspeedsetter.setDouble(0.0);
                    break;
                }
            }
            for (int i = 0; i < Constants.numberOfMotors; i++){
                motorSpeeds[i].setDouble(m_MotorBoard.getXSpeed(i));
            }

            //updates last variables based on shuffleboard
            lastDisable = false;
        }
        else{
            m_MotorBoard.setAllSpeed(0.0);
            lastDisable = true;
            if (currallspeedsetter != lastallspeedsetter){
                for (int i = 0; i < Constants.numberOfMotors; i++){
                    motorSpeeds[i].setDouble(allspeedsetter.get().getDouble());
                }
            }
            for (int i = 0; i < Constants.numberOfMotors; i++){
                if ((currSpeeds[i] != lastSpeeds[i]) && (syncButtons[i].get().getBoolean())){
                    for (int j = 0; j < Constants.numberOfMotors; j++){
                        if (syncButtons[j].get().getBoolean())
                            motorSpeeds[j].setDouble(motorSpeeds[i].get().getDouble());
                    }
                }
            }
            for (int i = 0; i < Constants.numberOfMotors-1; i++){
                if (motorSpeeds[i].get().getDouble() != motorSpeeds[i+1].get().getDouble()){
                    allspeedsetter.setDouble(0.0);
                    break;
                }
            }
        }
        for(int i = 0; i < lastSpeeds.length; i++)
            lastSpeeds[i] = motorSpeeds[i].get().getDouble();
        lastallspeedsetter = allspeedsetter.get().getDouble();
    }
}