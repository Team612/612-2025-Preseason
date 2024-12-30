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

        // this code makes it so that any invalid inputs to the max percent will be imediately corrected
        // !FUN FACT! it  bans value of zero in a unique way
        // it is banned as a variable but not on the display, because you often need have a zero at one point while typing in the value you want
        // this way the user can type in the value they want going through zero and it the code will not annoyingly correct it
        if (neoMotorMaxPercentChange.get().getDouble() <= 0){
            neoMotorMaxPercentChange.setDouble(0);
            Constants.maxNeoPercentChange = 0.001;
        }
        else if (cimMotorMaxPercentChange.get().getDouble() <= 0){
            cimMotorMaxPercentChange.setDouble(0);
            Constants.maxCimPercentChange = 0.001;
        }
        else{
            if (neoMotorMaxPercentChange.get().getDouble() > 1)
                neoMotorMaxPercentChange.setDouble(1.0);
            else if (cimMotorMaxPercentChange.get().getDouble() > 1)
                cimMotorMaxPercentChange.setDouble(0);
            Constants.maxNeoPercentChange = neoMotorMaxPercentChange.get().getDouble();
            Constants.maxCimPercentChange = cimMotorMaxPercentChange.get().getDouble();
        }

        // this code simply changes the motor inverse, no updates, or verification required. I wish the rest of the shuffleboard code was this simple 
        // but no. Shuffleboard has no method to detect user input, only a method to get the current value. Meaning I have to make this whole clusterfuck
        // of code which manually detects user inputs with currents and lasts and has a shit ton of exceptions because the all speed setter, specific
        // speed setter and disable button all affect each other in weird ass ways. I would've been done in like two hours if it just let me detect
        // specifically user inputs cuz then I would just immediately apply them and then update the values in shuffleboard. Beware, anyone attempting 
        // to decipher this code will not have a fun time. I would recommend starting with reading the MotorBoard subsystem first. That code is elegant,
        // beautiful and simple. Reading this code will rot your mind until you are a decrepit angry zombie.
        for (int i = 0; i < Constants.numberOfMotors; i++){
            m_MotorBoard.setInverse(i, inverseButtons[i].get().getBoolean());
        }

        // defines the current values in shuffleboard and adds them to variables 
        // because to see if the user has inputed something we just compare to see if the last has changed from the current
        double currallspeedsetter = allspeedsetter.get().getDouble();
        double[] currSpeeds = new double[Constants.numberOfMotors];
            for(int i = 0; i < currSpeeds.length; i++)
                currSpeeds[i] = motorSpeeds[i].get().getDouble();
        
        // huge block of code that only runs when code is not disabled
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
            // specifically sets the speeds right after it undisabled because
            // this edge case doesn't meet the normal conditions to set speed
            if (lastDisable){
                for (int i = 0; i < Constants.numberOfMotors; i++){
                    m_MotorBoard.setSpecificSpeed(i, currSpeeds[i]);
                }
            }

            // updates shufflboard based on real speed values which have been changed by now
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

        // this block of code only runs during disabled mode
        else{
            // makes everything stop
            m_MotorBoard.setAllSpeed(0.0);

            // makes last variable true so when it is undisabled the previous code will know it was just disabled
            lastDisable = true;

            // makes the all speed setter still work in disabled mode
            if (currallspeedsetter != lastallspeedsetter){
                for (int i = 0; i < Constants.numberOfMotors; i++){
                    motorSpeeds[i].setDouble(allspeedsetter.get().getDouble());
                }
            }

            // makes the sync button still work in disabled mode
            for (int i = 0; i < Constants.numberOfMotors; i++){
                if ((currSpeeds[i] != lastSpeeds[i]) && (syncButtons[i].get().getBoolean())){
                    for (int j = 0; j < Constants.numberOfMotors; j++){
                        if (syncButtons[j].get().getBoolean())
                            motorSpeeds[j].setDouble(motorSpeeds[i].get().getDouble());
                    }
                }
            }

            // makes all speed setter going to zero still work in disabled mode
            for (int i = 0; i < Constants.numberOfMotors-1; i++){
                if (motorSpeeds[i].get().getDouble() != motorSpeeds[i+1].get().getDouble()){
                    allspeedsetter.setDouble(0.0);
                    break;
                }
            }
        }

        // updates lasts so that the code knows when users input stuff
        for(int i = 0; i < lastSpeeds.length; i++)
            lastSpeeds[i] = motorSpeeds[i].get().getDouble();
        lastallspeedsetter = allspeedsetter.get().getDouble();
    }
}