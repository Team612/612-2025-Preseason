// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led_Colors extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  /** Creates a new Led_Colors. */
  public Led_Colors() {
      m_led = new AddressableLED(0);
      m_ledBuffer = new AddressableLEDBuffer(300); //300 should be the length
      m_led.setLength(m_ledBuffer.getLength());
      m_led.start(); 
  }

  public void setOneLed(int l, int r, int g, int b){
    m_ledBuffer.setRGB(l, r, g, b);
    m_led.setData(m_ledBuffer);
  }

  public void setAllLeds(int r, int g, int b){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, r, g, b);
      m_led.setData(m_ledBuffer);
    }
  }

  public void setGreen(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 0, 255, 0);
      m_led.setData(m_ledBuffer);
    }
  }

  public void setPink(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 255,20,147);
      m_led.setData(m_ledBuffer);
    }
  }

    public void setGreenAndPink(){
    boolean pink = true;
    int a = 0;
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      
      if(pink = true){
        a++;
        m_ledBuffer.setRGB(i, 255,20,147);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          pink = false;
        }
      }
      else{
        a++;
        m_ledBuffer.setRGB(i, 0, 255, 0);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          pink = true;
        }
      }
    }
  }


  public void setPurpleAndWhite(){
    boolean purple = true;
    int a = 0;
    
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      if(purple = true){
        a++;
        m_ledBuffer.setRGB(i,148,0,211);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          purple = false;
        }
      }
      else{
        a++;
        m_ledBuffer.setRGB(i, 255, 255, 255);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          purple = true;
        }
      }
    }
  }
  // length
  public int getLength(){
    return m_ledBuffer.getLength();
  }

  @Override
  public void periodic() {
        //Default (Not detecting anything):
          // 612 Colors/Chantilly Colors: Blue & Yellow, Purple & White
      
        

        // When Detects Only April Tags:
            // Green



        // When Sees Only Note:
            // Pink 



        // When sees note & april tags:
            // Green & Pink: Alternating green and pink around




  }
}
