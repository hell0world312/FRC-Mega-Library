// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MegaLibrary;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class led {
     //LED COMMANDS
    public void initializeLED(int length, AddressableLED led, AddressableLEDBuffer ledBuffer, int red, int green, int blue){
        ledBuffer = new AddressableLEDBuffer(length);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        for (int i = 0; i < ledBuffer.getLength(); i++)
        ledBuffer.setRGB(i, red, green, blue);

        led.start();
    }

    public void changeLEDColor(AddressableLED led, AddressableLEDBuffer ledBuffer, int red, int green, int blue){
        for (int i = 0; i < ledBuffer.getLength(); i++)
        ledBuffer.setRGB(i, red, green, blue);
        led.setData(ledBuffer);
    }

    public void sequenceLED(AddressableLED led, AddressableLEDBuffer ledBuffer, int red1, int green1, int blue1, int red2, int green2, int blue2){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 2 == 0){
               ledBuffer.setRGB(i, red1, green1, blue1);
              led.setData(ledBuffer); 
            }
            else{
                ledBuffer.setRGB(i, red2, green2, blue2);
              led.setData(ledBuffer);
            }
          }
    }

    public void clearLED(AddressableLED led, AddressableLEDBuffer ledBuffer){
        for (int i = 0; i < ledBuffer.getLength(); i++)
        ledBuffer.setRGB(i, 0, 0, 0);
        led.setData(ledBuffer);
    }

}
