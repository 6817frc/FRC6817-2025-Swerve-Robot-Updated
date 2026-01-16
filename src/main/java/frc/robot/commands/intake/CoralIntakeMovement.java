// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralIntakeMovement extends InstantCommand {

  public int timer;
  public AddressableLED m_LED;
  public AddressableLEDBuffer m_LedBuffer;
  public int m_rainbowHue;

  public CoralIntakeMovement(CoralIntake intake, AddressableLED led, AddressableLEDBuffer buffer) {
    m_LED = led;
    m_LedBuffer = buffer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public void rainbow() {
    for (var i = 0; i < m_LedBuffer.getLength(); i++) {
    final var hue = (m_rainbowHue + (i * 180 / m_LedBuffer.getLength())) % 100;
    m_LedBuffer.setHSV(i, hue, 255, 16);
  }
    m_rainbowHue += 3;
    m_rainbowHue %= 180;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  @Override
  public void execute() {
    rainbow();
    m_LED.setData(m_LedBuffer);
    timer++;
  }

  @Override
  public boolean isFinished() {
    if (timer > 10) {
      return true;
    } 
    return false;
  }
}
