// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  AddressableLED ledstrip;
  AddressableLEDBuffer m_ledBuffer;
  int k = 0;
  public LightsSubsystem() 
  {
    m_ledBuffer = new AddressableLEDBuffer(36);
    ledstrip = new AddressableLED(0);
    ledstrip.setLength(m_ledBuffer.getLength());
   

    ledstrip.setData(m_ledBuffer);
    ledstrip.start();
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSolidColor(Color8Bit color)
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setLED(i, color);
   }
   
   ledstrip.setData(m_ledBuffer);
  }

  public void TestColor()
  {
    m_ledBuffer.setLED(k, Color.kGold);
    k += 1;
    ledstrip.setData(m_ledBuffer);
  }
  public void setSectionColor(int firstLight, int lastLight, Color color)
  {
    for(int i = firstLight; i < lastLight; i++)
    {
      m_ledBuffer.setLED(i, color);
      System.out.println("fuck");
    }
    ledstrip.setData(m_ledBuffer);
  }
}
