package frc.robot;

import java.util.Map;

import javax.swing.text.TabExpander;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.CANSparkMaxSubsystem;
import frc.robot.subsystems.talonSubsystem;

public class ShuffleboardTabs {
    // defines tabs
    private ShuffleboardTab neoMotorsTab;
    private ShuffleboardTab simMotorsTab;
    
    // defines all of the buttons on the shuffleboard
    private GenericEntry[] motorId = new GenericEntry[4];
    private GenericEntry[] motorSpeeds = new GenericEntry[4];
    private GenericEntry[] syncButtons = new GenericEntry[4];
    private GenericEntry[] inverseButtons = new GenericEntry[4];
    private GenericEntry allspeedsetter;
    private GenericEntry disableButtons;

    private GenericEntry hi;

    // gets subsystem
    private CANSparkMaxSubsystem cansparkmax = CANSparkMaxSubsystem.getInstance();
    private talonSubsystem talonsubsystem = talonSubsystem.getInstance();

    //initializes all the buttons with default values
    public void initButton(){
        neoMotorsTab = Shuffleboard.getTab("Neo Motors Tab");
        simMotorsTab = Shuffleboard.getTab("Sim Motors Tab");
        allspeedsetter = neoMotorsTab.add("Set all CAN instance speeds", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).withPosition(0,0).withSize(3,1).getEntry();
        disableButtons = neoMotorsTab.add("Disable", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(3, 0).getEntry();
        hi = simMotorsTab.add("this is a test slider", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).withPosition(0,0).withSize(3,1).getEntry();
        for(int i = 0; i< 4; i++){
                motorId[i] = neoMotorsTab.add("Motor " + (i +1) + "CAN ID", i+1).withPosition(i,1).withSize(1,1).getEntry();
                motorSpeeds[i] = neoMotorsTab.add("Motor " + (i+1) + "Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(i,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                syncButtons[i] = neoMotorsTab.add("Sync Motor " + (i +1), false).withWidget(BuiltInWidgets.kToggleButton).withPosition(i, 3).getEntry();
                inverseButtons[i] = neoMotorsTab.add("Inverse " + (i +1), false).withWidget(BuiltInWidgets.kToggleButton).withPosition(i, 4).getEntry();
        }
    }

    // defines the last shuffleboard values in variables
    // because to see if the user has inputed something we just compare to see if the last has changed from the current
    private double[] lastIds = new double[4];
    private double[] lastSpeed = new double[4];
    double lastallspeedsetter = 0.0;
    boolean lastdisablebutton = false;

    // runs periodicly to check if a user has inputed something and then changes its actual value and updates shufflboard
    public void updateButtons(){
        talonsubsystem.setBobSpeed(hi.get().getDouble());
        // System.out.println(talonsubsystem.getBob().get());

        // defines the current values in shuffleboard and adds them to variables 
        // because to see if the user has inputed something we just compare to see if the last has changed from the current
        double currallspeedsetter = allspeedsetter.get().getDouble();
        boolean currdisablebutton = disableButtons.get().getBoolean();
        double[] currCanIds = new double[4];
        double[] currSpeed = new double[4];
        for(int i = 0; i < 4; i++){
            currCanIds[i] = motorId[i].get().getDouble();
            currSpeed[i] = motorSpeeds[i].get().getDouble();
        }

        // sets real values
        // checks if the user actually inputed something first so it doesn't spam inputs to real variables
        boolean keepCheckingForSync = true;
        boolean changeAllSpeedSetter = true;
        if (currdisablebutton != lastdisablebutton && disableButtons.get().getBoolean()){
            cansparkmax.setAllSpeed(0.0);
        }
        if (!disableButtons.get().getBoolean()){
            for (int i = 0; i < motorSpeeds.length; i++){
                if (syncButtons[i].get().getBoolean() && keepCheckingForSync && currSpeed[i]!=lastSpeed[i]){
                    changeAllSpeedSetter = false;
                    allspeedsetter.setDouble(0.0);
                    for(int j = 0; j < motorSpeeds.length; j++){
                        if (syncButtons[j].get().getBoolean())                        
                            if (inverseButtons[j].get().getBoolean() && i!=j)
                                cansparkmax.setMotorsBasedOnIndex(j,-motorSpeeds[i].get().getDouble());
                            else
                                cansparkmax.setMotorsBasedOnIndex(j,motorSpeeds[i].get().getDouble());
                    }
                    keepCheckingForSync = false;
                }
                else if (!syncButtons[i].get().getBoolean() && currSpeed[i]!=lastSpeed[i]){
                    cansparkmax.setMotorsBasedOnIndex(i,motorSpeeds[i].get().getDouble());
                    changeAllSpeedSetter = false;
                    allspeedsetter.setDouble(0.0);
                }
            }
        }
        if (lastallspeedsetter!=currallspeedsetter && changeAllSpeedSetter && !disableButtons.get().getBoolean()){
            cansparkmax.setAllSpeed(allspeedsetter.get().getDouble());
        }
        else if (lastallspeedsetter!=currallspeedsetter && changeAllSpeedSetter && disableButtons.get().getBoolean()){
            for (int i = 0; i < motorSpeeds.length; i++){
                motorSpeeds[i].setDouble(currallspeedsetter);
            }
        }
        if (currdisablebutton != lastdisablebutton && !disableButtons.get().getBoolean()){
            cansparkmax.setAllSpeed(allspeedsetter.get().getDouble());
            for (int i = 0; i < motorSpeeds.length; i++){
                cansparkmax.setMotorsBasedOnIndex(i,motorSpeeds[i].get().getDouble());
            }
        }
        for(int i = 0; i < 4; i++){
            if(currCanIds[i] != lastIds[i]){
                cansparkmax.setID(i, (int) Math.round(currCanIds[i]));
            }
        }
        
        // updates shufflboard based on real values which have been changed by now unless disabled mode is on
        for(int i = 0; i < 4; i++){
            motorId[i].setDouble(cansparkmax.getXMotor(i).getDeviceId());
            if (!disableButtons.get().getBoolean())
                motorSpeeds[i].setDouble(cansparkmax.getXMotor(i).get());
        }

        //updates last variables based on shuffleboard
        for(int i = 0; i < 4; i++){
            lastIds[i] = motorId[i].get().getDouble();
            lastSpeed[i] = motorSpeeds[i].get().getDouble();
        }
        lastallspeedsetter = allspeedsetter.get().getDouble();
        lastdisablebutton = disableButtons.get().getBoolean();
    }
}
