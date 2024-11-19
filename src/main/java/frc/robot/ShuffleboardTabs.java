package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.CANSparkMaxSubsystem;

public class ShuffleboardTabs {
    private ShuffleboardTab motorsTab;

    private GenericEntry motor4;
    private GenericEntry motor2;
    private GenericEntry motor3;
    private GenericEntry motor1;
    private GenericEntry allspeedsetter;

    private CANSparkMaxSubsystem cansparkmax = CANSparkMaxSubsystem.getInstance();

    public void initButton(){
        motorsTab = Shuffleboard.getTab("Motors Tab");

        motorsTab.add("Cool Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        allspeedsetter = motorsTab.add("Set all CAN instance speeds", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        motor1 = motorsTab.add("Motor 1 CAN ID", 0).getEntry();
        motor2 = motorsTab.add("Motor 2 CAN ID", 0).getEntry();
        motor3 = motorsTab.add("Motor 3 CAN ID", 0).getEntry();
        motor4 = motorsTab.add("Motor 4 CAN ID", 0).getEntry();
    }

    double last1 = 0.0;
    double last2 = 0.0;
    double last3 = 0.0;
    double last4 = 0.0;
    double lastallspeedsetter = 0.0;
    public void updateButtons(){
        double curr1 = motor1.get().getDouble();
        double curr2 = motor2.get().getDouble();
        double curr3 = motor3.get().getDouble();
        double curr4 = motor4.get().getDouble();
        double currallspeedsetter = allspeedsetter.get().getDouble();

        if (last1!=curr1)
            cansparkmax.setID(0,(int)Math.round(motor1.get().getDouble()));
        if (last2!=curr2)
            cansparkmax.setID(1,(int)Math.round(motor2.get().getDouble()));
        if (last3!=curr3)
            cansparkmax.setID(2,(int)Math.round(motor3.get().getDouble()));
        if (last4!=curr4)
            cansparkmax.setID(3,(int)Math.round(motor4.get().getDouble()));
        if (currallspeedsetter != lastallspeedsetter){
            cansparkmax.setAllSpeed(allspeedsetter.get().getDouble());
        }
        

        motor1.setDouble(cansparkmax.getXMotor(0).getDeviceId());
        motor2.setDouble(cansparkmax.getXMotor(1).getDeviceId());
        motor3.setDouble(cansparkmax.getXMotor(2).getDeviceId());
        motor4.setDouble(cansparkmax.getXMotor(3).getDeviceId());

        last1 = motor1.get().getDouble();
        last2 = motor2.get().getDouble();
        last3 = motor3.get().getDouble();
        last4 = motor4.get().getDouble();
        lastallspeedsetter = allspeedsetter.get().getDouble();
    }
}
