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
    private GenericEntry motorId4;
    private GenericEntry motorId2;
    private GenericEntry motorId3;
    private GenericEntry motorId1;
    private GenericEntry allspeedsetter;
    private GenericEntry motorSpeed1;
    private GenericEntry motorSpeed2;
    private GenericEntry motorSpeed3;
    private GenericEntry motorSpeed4;
    private GenericEntry syncButton1;
    private GenericEntry syncButton2;
    private GenericEntry syncButton3;
    private GenericEntry syncButton4;

    private CANSparkMaxSubsystem cansparkmax = CANSparkMaxSubsystem.getInstance();

    public void initButton(){
        motorsTab = Shuffleboard.getTab("Motors Tab");

      //  motorsTab.add("Cool Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        allspeedsetter = motorsTab.add("Set all CAN instance speeds", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).withPosition(0,0).withSize(3,1).getEntry();
        motorId1 = motorsTab.add("Motor 1 CAN ID", 0).withPosition(0,1).withSize(1,1).getEntry();
        motorId2 = motorsTab.add("Motor 2 CAN ID", 0).withPosition(1, 1).withSize(1, 1).getEntry();
        motorId3 = motorsTab.add("Motor 3 CAN ID", 0).withPosition(2,1).withSize(1,1).getEntry();
        motorId4 = motorsTab.add("Motor 4 CAN ID", 0).withPosition(3,1).withSize(1,1).getEntry();
        motorSpeed1 = motorsTab.add("Motor 1 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed2 = motorsTab.add("Motor 2 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(1,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed3 = motorsTab.add("Motor 3 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(2,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed4 = motorsTab.add("Motor 4 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(3,2).withSize(1,1).withProperties(Map.of("min", -1, "max", 1)).getEntry();

        syncButton1 = motorsTab.add("Sync Motor 1", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 3).getEntry();
        syncButton2 = motorsTab.add("Sync Motor 2", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(1,3).getEntry();
        syncButton3 = motorsTab.add("Sync Motor 3", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(2,3).getEntry();
        syncButton4 = motorsTab.add("Sync Motor 4", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(3, 3).getEntry();
    }

    double lastId1 = 0.0;
    double lastId2 = 0.0;
    double lastId3 = 0.0;
    double lastId4 = 0.0;
    double lastSpeed1 = 0.0;
    double lastSpeed2 = 0.0;
    double lastSpeed3 = 0.0;
    double lastSpeed4 = 0.0;
    double lastallspeedsetter = 0.0;

    private boolean syncMotor1 = false;
    private boolean syncMotor2 = false;
    private boolean syncMotor3 = false;
    private boolean syncMotor4 = false;

    public void updateButtons(){
        double currCanId1 = motorId1.get().getDouble();
        double currCanId2 = motorId2.get().getDouble();
        double currCanId3 = motorId3.get().getDouble();
        double currCanId4 = motorId4.get().getDouble();
        double currallspeedsetter = allspeedsetter.get().getDouble();
        double currSpeed1 = motorSpeed1.get().getDouble();
        double currSpeed2 = motorSpeed2.get().getDouble();
        double currSpeed3 = motorSpeed3.get().getDouble();
        double currSpeed4 = motorSpeed4.get().getDouble();

        syncMotor1 = syncButton1.get().getBoolean();
        syncMotor2 = syncButton2.get().getBoolean();
        syncMotor3 = syncButton3.get().getBoolean();
        syncMotor4 = syncButton4.get().getBoolean();
        // rather poorway to handle syncing
        //did this so that I can use any slider to control te synced motors, previously only the motor with the smallest can id would work
        //so if 1 & 4 were selected you could only move both by moving one
        //this is laggy as hell because of the averageing
        double totalSpeed = 0.0;
        int syncedCount = 0;
        
        if (syncMotor1) {
            totalSpeed += currSpeed1;
            syncedCount++;
        }
        if (syncMotor2) {
            totalSpeed += currSpeed2;
            syncedCount++;
        }
        if (syncMotor3) {
            totalSpeed += currSpeed3;
            syncedCount++;
        }
        if (syncMotor4) {
            totalSpeed += currSpeed4;
            syncedCount++;
        }
        
        Double referenceSpeed = (syncedCount > 0) ? totalSpeed / syncedCount : null;

        if(referenceSpeed != null){
            
            if (syncMotor1 && currSpeed1 != referenceSpeed) motorSpeed1.setDouble(referenceSpeed);
            if (syncMotor2 && currSpeed2 != referenceSpeed) motorSpeed2.setDouble(referenceSpeed);
            if (syncMotor3 && currSpeed3 != referenceSpeed) motorSpeed3.setDouble(referenceSpeed);
            if (syncMotor4 && currSpeed4 != referenceSpeed) motorSpeed4.setDouble(referenceSpeed);
            
            if (syncMotor1) cansparkmax.setMotorsBasedOnCan((int) currCanId1, referenceSpeed);
            if (syncMotor2) cansparkmax.setMotorsBasedOnCan((int) currCanId2, referenceSpeed);
            if (syncMotor3) cansparkmax.setMotorsBasedOnCan((int) currCanId3, referenceSpeed);
            if (syncMotor4) cansparkmax.setMotorsBasedOnCan((int) currCanId4, referenceSpeed);
    }   

        
        if (lastId1!=currCanId1)
            cansparkmax.setID(0,(int)Math.round(motorId1.get().getDouble()));
        if (lastId2!=currCanId2)
            cansparkmax.setID(1,(int)Math.round(motorId2.get().getDouble()));
        if (lastId3!=currCanId3)
            cansparkmax.setID(2,(int)Math.round(motorId3.get().getDouble()));
        if (lastId4!=currCanId4)
            cansparkmax.setID(3,(int)Math.round(motorId4.get().getDouble()));
        if (currallspeedsetter != lastallspeedsetter){
            cansparkmax.setAllSpeed(allspeedsetter.get().getDouble());
            motorSpeed1.setDouble(currallspeedsetter);
            motorSpeed2.setDouble(currallspeedsetter);
            motorSpeed3.setDouble(currallspeedsetter);
            motorSpeed4.setDouble(currallspeedsetter);         
        }
        
        if(!syncMotor1 && currSpeed1 != lastSpeed1){
            cansparkmax.setMotorsBasedOnCan((int)currCanId1, motorSpeed1.get().getDouble());
        }
        if(!syncMotor2 && currSpeed2 != lastSpeed2){
            cansparkmax.setMotorsBasedOnCan((int)currCanId2, motorSpeed2.get().getDouble());
        }
        if(!syncMotor3 && currSpeed3 != lastSpeed3){
            cansparkmax.setMotorsBasedOnCan((int)currCanId3, motorSpeed3.get().getDouble());
        }
        if(!syncMotor4 && currSpeed4 != lastSpeed4){
            cansparkmax.setMotorsBasedOnCan((int)currCanId4, motorSpeed4.get().getDouble());
        }

        motorId1.setDouble(cansparkmax.getXMotor(0).getDeviceId());
        motorId2.setDouble(cansparkmax.getXMotor(1).getDeviceId());
        motorId3.setDouble(cansparkmax.getXMotor(2).getDeviceId());
        motorId4.setDouble(cansparkmax.getXMotor(3).getDeviceId());

        lastId1 = motorId1.get().getDouble();
        lastId2 = motorId2.get().getDouble();
        lastId3 = motorId3.get().getDouble();
        lastId4 = motorId4.get().getDouble();
        lastallspeedsetter = allspeedsetter.get().getDouble();
        lastSpeed1 = motorSpeed1.get().getDouble();
        lastSpeed2 = motorSpeed2.get().getDouble();
        lastSpeed3 = motorSpeed3.get().getDouble();
        lastSpeed4 = motorSpeed4.get().getDouble();



    }
}
