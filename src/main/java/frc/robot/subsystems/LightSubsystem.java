// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    AddressableLED leds = new AddressableLED(5);
    AddressableLEDBuffer _ledBuffer = new AddressableLEDBuffer(60);
    private int _rainbowFirstPixelHue;
    
  /** Creates a new ExampleSubsystem. */
  public LightSubsystem() {
    leds.setLength(_ledBuffer.getLength());
  }

  public void greenLights(){
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        _ledBuffer.setRGB(i, 0, 255, 0);
     }
     
     leds.setData(_ledBuffer);
  }
  public void rainbowLights(){
    // For every pixel
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (_rainbowFirstPixelHue + (i * 180 / _ledBuffer.getLength())) % 180;
        // Set the value
        _ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      _rainbowFirstPixelHue += 3;
      // Check bounds
      _rainbowFirstPixelHue %= 180;
     
     leds.setData(_ledBuffer);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
