package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.CANSparkMaxSubsystem;

public class ShuffleboardTabs {
    private ShuffleboardTab motorsTab;

    private GenericEntry motorId4;
    private GenericEntry motorId2;
    private GenericEntry motorId3;
    private GenericEntry motorId1;
    private GenericEntry allspeedsetter;
    private GenericEntry motorSpeed1;
    private GenericEntry motorSpeed2;
    private GenericEntry motorSpeed3;
    private GenericEntry motorSpeed4;

    private CANSparkMaxSubsystem cansparkmax = CANSparkMaxSubsystem.getInstance();

    public void initButton(){
        motorsTab = Shuffleboard.getTab("Motors Tab");

        motorsTab.add("Cool Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        allspeedsetter = motorsTab.add("Set all CAN instance speeds", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorId1 = motorsTab.add("Motor 1 CAN ID", 0).getEntry();
        motorId2 = motorsTab.add("Motor 2 CAN ID", 0).getEntry();
        motorId3 = motorsTab.add("Motor 3 CAN ID", 0).getEntry();
        motorId4 = motorsTab.add("Motor 4 CAN ID", 0).getEntry();
        motorSpeed1 = motorsTab.add("Motor 1 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed2 = motorsTab.add("Motor 2 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed3 = motorsTab.add("Motor 3 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motorSpeed4 = motorsTab.add("Motor 4 Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();

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
        }
        
        if(currSpeed1 != lastSpeed1){
            cansparkmax.setMotorsBasedOnCan((int)currCanId1, motorSpeed1.get().getDouble());
        }
        if(currSpeed2 != lastSpeed2){
            cansparkmax.setMotorsBasedOnCan((int)currCanId2, motorSpeed2.get().getDouble());
        }
        if(currSpeed3 != lastSpeed3){
            cansparkmax.setMotorsBasedOnCan((int)currCanId3, motorSpeed3.get().getDouble());
        }
        if(currSpeed4 != lastSpeed4){
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
