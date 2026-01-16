// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Value;

import com.fasterxml.jackson.annotation.JsonIncludeProperties.Value;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.CoralIntakeMovement;
import com.revrobotics.spark.SparkBase.ResetMode;


public class CoralIntake extends SubsystemBase {
  /** Creates a new Intake. */

  //These are the motors used in the subsystem.
  public final AddressableLED m_LED;
  public final AddressableLEDBuffer m_LedBuffer;
  public final SparkMax m_intakeWheels;
  public final SparkMax m_intakeWrist;
	public final SparkMax m_intakeArm1;
  public final SparkClosedLoopController armClosedLoopController;
  public final SparkClosedLoopController wristClosedLoopController;
  public final AbsoluteEncoder armEncoder;
  public final AbsoluteEncoder wristEncoder;
  public final SparkMax m_intakeArm2;
  // public final SparkClosedLoopController armPID;

  public CoralIntake() {
    m_LED = new AddressableLED(Ports.PWM.LED_STRIP);
    m_LedBuffer = new AddressableLEDBuffer(8);
    m_LED.setLength(m_LedBuffer.getLength());
    m_LED.setData(m_LedBuffer);
    m_LED.start();

    //This sets the configuration for the motor controlling the wheels of the intake subsystem
    m_intakeWheels = new SparkMax(Ports.CAN.IntakeWheels, MotorType.kBrushless); //TODO update for new motors
    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(false).idleMode(IdleMode.kBrake); //TODO update for new motors

    //This sets the config for the motor controlling the additional arm movement
    m_intakeWrist = new SparkMax(Ports.CAN.Wrist, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig.inverted(true).idleMode(IdleMode.kBrake);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.8, 0.0, 0.0, ClosedLoopSlot.kSlot0)
    .pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot1).maxOutput(0.15, ClosedLoopSlot.kSlot0).minOutput(-0.15, ClosedLoopSlot.kSlot0);
    wristClosedLoopController = m_intakeWrist.getClosedLoopController();
    wristEncoder = m_intakeWrist.getAbsoluteEncoder();

    /*This sets the configuration for other motor controlling the overall arm movement
    This motor is set to follow the other arm motor and is reversed because they are on opposite sides */
    m_intakeArm2 = new SparkMax(Ports.CAN.Arm2, MotorType.kBrushless);
    SparkMaxConfig arm2Config = new SparkMaxConfig();
    arm2Config.inverted(true).idleMode(IdleMode.kBrake);
    arm2Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(3.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
    .pid(1.0, 0.0, 0.0, ClosedLoopSlot.kSlot1); //TODO update for new motors
    armClosedLoopController = m_intakeArm2.getClosedLoopController();
    armEncoder = m_intakeArm2.getAbsoluteEncoder();

    //This sets the configuration for one motor controlling the overall arm movement
    m_intakeArm1 = new SparkMax(Ports.CAN.Arm1, MotorType.kBrushless); //TODO update for new motors
    SparkMaxConfig arm1Config = new SparkMaxConfig();
    arm1Config.follow(m_intakeArm2, true).idleMode(IdleMode.kBrake);

    // armPID.setOutputRange(-0.5, 0.25);  //TODO doesn't work anymore so find out how to limit motor output
    m_intakeWheels.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeArm1.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeArm2.configure(arm2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    double value = SmartDashboard.getNumber("PValue", 3);
    SmartDashboard.putNumber("PValue", value);
  }
    public double L1Position = 0.44; //TODO change to real value
    public double L2Position = 0.30; //wrist:0.30
    public double L3Position = 0.30; //wrist:0.43
    double L4Position = 0.35; //TODO change to real value //wwrist:
    public double L1WristPosition = 0.58;
    public double L2WristPosition = 0.3;
    public double L3WristPosition = 0.41;
    //double L4WristPosition = 0;
    public double safePosition = 0.51; //wrist:0.23
    public double intakePosition = 0.30; //wrist:0.37
    boolean armManualMode = false;
    boolean wristManualMode = false;
    boolean leftManualMode = false;
    boolean rightManualMode = false;

    /* Functions for various arm movements: */

    //moves arm based on joystick input
    public void armMove(double LeftYValue) {
      if(armEncoder.getPosition() > 0.196) {
        if (LeftYValue != 0) {
        armManualMode = true;
        armClosedLoopController.setReference(0.3 * LeftYValue, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } else if (armManualMode == true && LeftYValue == 0) {
        armClosedLoopController.setReference(0.0, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        armClosedLoopController.setReference(armEncoder.getPosition(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        armManualMode = false;
        }
      } else {
        armClosedLoopController.setReference(0.2, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
    }
    //moves arm to to score on level 1
    public void armL1() {
      armClosedLoopController.setReference(L1Position, SparkMax.ControlType.kPosition);
      wristClosedLoopController.setReference(L1WristPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    //moves arm to score on level 2
    public void armL2() {
      armClosedLoopController.setReference(L2Position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      wristClosedLoopController.setReference(L2WristPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }
    //moves arm to score on level 3
    public void armL3() {
      armClosedLoopController.setReference(L3Position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      wristClosedLoopController.setReference(L3WristPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    //moves arm to score on level 4
    public void armL4() {
      armClosedLoopController.setReference(L4Position, SparkMax.ControlType.kPosition);
    }
    //moves arm to a position where it is safe to travel across the field
    public void armSafe() {
      armClosedLoopController.setReference(safePosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      wristClosedLoopController.setReference(0.23, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
//intake from Human Player Station
    public void armIntake() {
      armClosedLoopController.setReference(intakePosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      wristClosedLoopController.setReference(0.37, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    //stops all arm movement
    public void stopIntake() {
      m_intakeArm2.set(0);
    }

    /* Functions for various wrist movements: */

    //moves wrist using joystick input
    public void wristMove(double RightYValue) {
      if (wristEncoder.getPosition() > 0.57 && armEncoder.getPosition() <= 0.20) {
        wristClosedLoopController.setReference(0.55, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      } else {
        if (RightYValue != 0) {
        wristManualMode = true;
        wristClosedLoopController.setReference(0.5 * RightYValue, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } else if (wristManualMode == true && RightYValue == 0) {
        wristClosedLoopController.setReference(0.0, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        wristClosedLoopController.setReference(wristEncoder.getPosition(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        wristManualMode = false;
        }
      }
    }

    //stops all wrist movement
    public void stopWrist() {
      m_intakeWrist.set(0);
    }
    //moves wheels for intake
    public void wheelIn(double rightTrig) {
      if (rightTrig != 0) {
        rightManualMode = true;
        m_intakeWheels.set(0.25 * rightTrig);
      } else if (rightManualMode == true && rightTrig == 0) {
        m_intakeWheels.set(0);
        rightManualMode = false;
      }
    }
    //moves wheels for outake
    public void wheelOut(double leftTrig) {
      if (leftTrig != 0) {
        leftManualMode = true;
        m_intakeWheels.set(-0.1 * leftTrig);
      } else if (leftManualMode == true && leftTrig ==0) {
        m_intakeWheels.set(0);
        leftManualMode = false;
      }
    }
    //moves wheels for outake
    public void wheelOutTest(double speedValue) {
      m_intakeWheels.set(-speedValue);
    }
    public void wheelOutFast() {
      m_intakeWheels.set(-1.0);
    }
    //stops all wheel movement
    public void stopWheels() {
      m_intakeWheels.set(0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}