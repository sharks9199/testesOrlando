// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LimelightConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer ledBuffer;
  private final AddressableLEDBufferView ledLeft, ledRight, ledMiddleLeft, ledMiddleRight;
  
  public LEDSubsystem() { 
    m_led = new AddressableLED(8);

    ledBuffer = new AddressableLEDBuffer(79);

    ledLeft = ledBuffer.createView(0, 28);
    ledMiddleRight = ledBuffer.createView(32, 39);
    ledMiddleLeft = ledBuffer.createView(40, 46);
    ledRight = ledBuffer.createView(50, 78).reversed();

    m_led.setLength(ledBuffer.getLength());
    m_led.setData(ledBuffer);
    m_led.start();
  }

  public void setPattern(LEDPattern ledPattern){
    ledPattern.applyTo(ledRight);
    ledPattern.applyTo(ledLeft);

    // Write the data to the LED strip
    m_led.setData(ledBuffer);
  }

  public void setMidLeftPattern(LEDPattern ledPattern){
    ledPattern.applyTo(ledMiddleLeft);

    // Write the data to the LED strip
    m_led.setData(ledBuffer);
  }

  public void setMidRightPattern(LEDPattern ledPattern){
    ledPattern.applyTo(ledMiddleRight);

    // Write the data to the LED strip
    m_led.setData(ledBuffer);
  }

  public void setTurboPattern(double speed){
    LEDConstants.speed = speed;

    LEDConstants.turboDisplay.applyTo(ledRight);
    LEDConstants.turboDisplay.applyTo(ledLeft);

    //System.out.println("Turbo ON");
    m_led.setData(ledBuffer);
  }

  public double updateValue(double speed){
    return speed;
  }

  @Override
  public void periodic() {
    //setPattern(LEDPattern.solid(Color.kGreen));

    setMidLeftPattern(LimelightHelpers.getFiducialID(LimelightConstants.LimelightReefLeft) > 0 ? LEDConstants.baseGreen : LEDConstants.baseBlack);
    setMidRightPattern(LimelightHelpers.getFiducialID(LimelightConstants.LimelightReefRight) > 0 ? LEDConstants.baseGreen : LEDConstants.baseBlack);
    
    //m_led.setData(ledBuffer);

  }
}
