// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Climber extends SubsystemBase {
  public final SparkMax m_climb;
  // public final RelativeEncoder frontEncoder;
  // public final RelativeEncoder backEncoder;
  // private final DigitalInput limitSwitch = new DigitalInput(1);
  private double frontEncoderOffset;
  private double backEncoderOffset;
  private double realMotorPos;
  private boolean leftManualMode;
  private boolean RightManualMode;

  /** Creates a new Climber. */
  public Climber() {
    m_climb = new SparkMax(Ports.CAN.Climb, MotorType.kBrushless); //TODO update for new motors
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.inverted(false).idleMode(IdleMode.kBrake);
    double value = SmartDashboard.getNumber("climbPValue", 0.05);
    SmartDashboard.putNumber("climbPValue", value);

    m_climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // frontEncoder = m_climbFront.getEncoder(); //this was used last year
    // backEncoder = m_climbBack.getEncoder();
    // frontEncoderOffset = 0;
    // backEncoderOffset = 0;
    // m_climbBack.follow(m_climbFront); // TODO check whether this needs to be used
  }

  public void climbMove(double coLeftTrig) {
    //set speed:0.35
    if (coLeftTrig != 0) {
      leftManualMode = true;
      m_climb.set(coLeftTrig);
    } else if (leftManualMode == true && coLeftTrig ==0) {
      m_climb.set(0);
      leftManualMode = false;
    }
  }

  public void climbMoveRev(double coRightTrig) {
    //set speed:-0.35
    if (coRightTrig != 0) {
      RightManualMode = true;
      m_climb.set(-coRightTrig);
    } else if (RightManualMode == true && coRightTrig ==0) {
      m_climb.set(0);
      RightManualMode = false;
    }
  }

  public void stopClimb() {
    m_climb.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
