package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.CANSparkMaxSubsystem;

public class ShuffleboardTabs {
    private ShuffleboardTab motorsTab;
    //should be cleaned up
    
    private GenericEntry[] motorId = new GenericEntry[4];
    private GenericEntry[] motorSpeeds = new GenericEntry[4];
    private GenericEntry[] syncButtons = new GenericEntry[4];
    private GenericEntry allspeedsetter;

   
    private CANSparkMaxSubsystem cansparkmax = CANSparkMaxSubsystem.getInstance();

    public void initButton(){
        motorsTab = Shuffleboard.getTab("Motors Tab");

      //  motorsTab.add("Cool Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        allspeedsetter = motorsTab.add("Set all CAN instance speeds", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).withPosition(0,0).withSize(3,1).getEntry();
        for(int i = 0; i< 4; i++){
                motorId[i] = motorsTab.add("Motor " + (i +1) + "CAN ID", i+1).withPosition(i,1).withSize(1,1).getEntry();
                motorSpeeds[i] = motorsTab.add("Motor " + (i+1) + "Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(i,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                syncButtons[i] = motorsTab.add("Sync Motor " + (i +1), false).withWidget(BuiltInWidgets.kToggleButton).withPosition(i, 3).getEntry();


        }
    }
    private double[] lastIds = new double[4];
    private double[] lastSpeeds = new double[4];
    private boolean[] lastSyncMotors = new boolean[4];

    double lastallspeedsetter = 0.0;


    public void updateButtons(){
     
        double currallspeedsetter = allspeedsetter.get().getDouble();
        double[] currCanIds = new double[4];
        double[] currSpeeds = new double[4];
        boolean[] currSyncMotors = new boolean[4];
        // start the syncing stuff, basically everytime a sync button is checked it adds the motors speed to total speed, and then later on it divides by amount of motors in order to sync them
        // ^ needs some work, there is definetly a better way to handle this
        double totalSpeed = 0.0;
        int syncedCount = 0;
        //get current values
        for(int i = 0; i < 4; i++){
            currCanIds[i] = motorId[i].get().getDouble();
            currSpeeds[i] = motorSpeeds[i].get().getDouble();
            currSyncMotors[i] = syncButtons[i].get().getBoolean();
            //to count refrence speed
            if (currSyncMotors[i]){
                totalSpeed += currSpeeds[i];
                syncedCount++;
            }
        }
        
        //syncing
        Double referenceSpeed = (syncedCount > 0) ? totalSpeed / syncedCount : null;
        if(referenceSpeed != null){
            for(int i = 0; i < 4; i++){
                if(currSyncMotors[i] && currSpeeds[i] != referenceSpeed){
                    motorSpeeds[i].setDouble(referenceSpeed);
                }
                if(currSyncMotors[i]) {
                    cansparkmax.setMotorsBasedOnCan((int)currCanIds[i], referenceSpeed);
                }
            }
        }
        //ids
        for(int i = 0; i < 4; i++){
            if(currCanIds[i] != lastIds[i]){
                cansparkmax.setID(i, (int) Math.round(currCanIds[i]));
            }
        }
        //individuals
        for(int i = 0; i < 4; i++){
            if(!currSyncMotors[i] && currSpeeds[i] != lastSpeeds[i]){
                motorSpeeds[i].setDouble(currSpeeds[i]);                
                cansparkmax.setMotorsBasedOnCan((int) currCanIds[i], currSpeeds[i]);
            }
        }
        
        

       //ids
       //all setter
       //individuals
       
        if (currallspeedsetter != lastallspeedsetter){
       
            cansparkmax.setAllSpeed(currallspeedsetter);
            for(int i = 0; i < 4; i++){
            motorSpeeds[i].setDouble(currallspeedsetter);
            lastSpeeds[i] = currallspeedsetter;

        }
            
        }
        
        
        for(int i = 0; i < 4; i++){
        motorId[i].setDouble(cansparkmax.getXMotor(i).getDeviceId());
        }    

        for(int i = 0; i < 4; i++){
            lastIds[i] = currCanIds[i];
            lastSpeeds[i] = currSpeeds[i];
            lastSyncMotors[i] = currSyncMotors[i];
        }
        lastallspeedsetter = currallspeedsetter;
     


    }
}
