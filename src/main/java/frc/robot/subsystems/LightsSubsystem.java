// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  AddressableLED ledstrip;
  AddressableLEDBuffer m_ledBuffer;

  public LightsSubsystem() 
  {
    m_ledBuffer = new AddressableLEDBuffer(LightConstants.numLEDGroups);
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

  public void setSectionColor(int firstLight, int lastLight, Color8Bit color)
  {
    for(int i = firstLight; i < lastLight; i++)
    {
      m_ledBuffer.setLED(i, color);
    }
    ledstrip.setData(m_ledBuffer);
  }

  public void setPattern(Color8Bit[] pattern) {
    if (pattern.length != m_ledBuffer.getLength()) throw new Error("Pattern length must match number of LEDs");

    for (int i = 0; i < pattern.length; i++) {
      m_ledBuffer.setLED(i, pattern[i]);
    }
    ledstrip.setData(m_ledBuffer);
  }

  public Color8Bit[] interpolatePatterns(Color8Bit[] pattern1, Color8Bit[] pattern2, double t) {
    if (pattern1.length != pattern2.length) throw new Error("Pattern lengths must match");

    Color8Bit[] result = new Color8Bit[pattern1.length];
    
    for (int i = 0; i < pattern1.length; i++) {
      int r = (int)(pattern1[i].red*(1-t) + pattern2[i].red*t);
      int g = (int)(pattern1[i].green*(1-t) + pattern2[i].green*t);
      int b = (int)(pattern1[i].blue*(1-t) + pattern2[i].blue*t);

      result[i] = new Color8Bit(r, g, b);
    }

    return result;
  }

  public Color8Bit[] shiftPattern(Color8Bit[] pattern) {
    Color8Bit[] result = new Color8Bit[pattern.length];
    result[0] = pattern[pattern.length-1];

    for (int i = 1; i < pattern.length; i++) {
      result[i] = pattern[i-1];
    }
    return result;
  }

  public Color8Bit[] rearrangePattern(Color8Bit[] pattern, int[] indices) {
    Color8Bit[] result = new Color8Bit[pattern.length];

    for (int i = 0; i < pattern.length; i++) {
      result[i] = pattern[indices[i]];
    }

    return result;
  }
}
