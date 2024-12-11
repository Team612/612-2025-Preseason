// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class WestCoastDrive extends SubsystemBase {
  /** Creates a new WestCoastDrive. */
private final WPI_TalonSRX left1;
private final WPI_TalonSRX left2;
private final WPI_TalonSRX right1;
private final WPI_TalonSRX right2;
private final DifferentialDrive drive;
public WestCoastDrive() {
    left1 = new WPI_TalonSRX(0);
    left2 = new WPI_TalonSRX(1);
    right1 = new WPI_TalonSRX(2);
    right2 = new WPI_TalonSRX(3);

    left1.follow(left2);
    right1.follow(right2);

    drive = new DifferentialDrive(left1::set, right2::set);
}

  public void drive(double ForwardY, double RotationZ){
    drive.arcadeDrive(ForwardY, RotationZ);
    
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

}
