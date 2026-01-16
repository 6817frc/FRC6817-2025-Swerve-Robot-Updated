// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeL2 extends InstantCommand {

  public CoralIntake intake;
  public AddressableLED m_LED;
  public AddressableLEDBuffer m_LedBuffer;
  public boolean inPosition;
  public double deadband = 0.01;

  public IntakeL2(CoralIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }  

  // public void isPosition() {
  //   if () {

  //     inPosition = true
  //   } else {
  //     inPosition = false;
  //   }
  // }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.armL2();
  }

  // @Override
  // public boolean isFinished() {
  //   if () {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      intake.stopIntake();
      intake.stopWrist();
    } else {

    }
  }
}
