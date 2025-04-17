// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.awt.Color;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm.Arm;

/** Add your docs here. */
public class LEDS {
  private AddressableLED leds;
  private AddressableLEDBuffer buffer;
  private static boolean isAuto = DriverStation.isAutonomous();
  private static boolean isTeleop = DriverStation.isTeleop();

  public LEDS(int port, int length) {
    leds = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    
    

  }

  public void RunLEDS() { 
    isTeleop = DriverStation.isTeleop();
    if (DriverStation.isDisabled()) {
      LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, edu.wpi.first.wpilibj.util.Color.kRed,edu.wpi.first.wpilibj.util.Color.kBlack);
      LEDPattern wave = base.breathe(Seconds.of(5));
      wave.applyTo(buffer);
      leds.setData(buffer);
    }
    if (DriverStation.isEnabled()) {
      Distance ledSpacing = Meters.of(1/ 120);
      LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, edu.wpi.first.wpilibj.util.Color.kBlack,edu.wpi.first.wpilibj.util.Color.kRed);
      LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
      pattern.applyTo(buffer);
      leds.setData(buffer);
    }
    
    
    // example
    // for (var i = 0; i < buffer.getLength(); i++) {
    //   // Sets the specified LED to the HSV values for red
    //   buffer.setHSV(i, 0, 100, 50);
    // }
    // System.out.println("set LEDs");
    // leds.setData(buffer);

  }
}
