// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class talonSubsystem extends SubsystemBase {
  
  private static talonSubsystem talonsubsystem = null;
  private TalonFX bob;

  public static talonSubsystem getInstance(){
    if (talonsubsystem == null){
      talonsubsystem = new talonSubsystem();
    }
    return talonsubsystem;
  }

  public TalonFX getBob(){
    return bob;
  }

  public talonSubsystem() {
    bob = new TalonFX(1);
  }


  public void setBobSpeed(double speed){
    
    bob.set(speed);
    
  }


  @Override
  public void periodic() {
    // System.out.println(bob.get());
  }
}
