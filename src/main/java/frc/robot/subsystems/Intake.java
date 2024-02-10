// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // motors and devices
  TalonFX intakeLiftRight = new TalonFX(Constants.rightIntakeLiftId);
  TalonFX intakeLiftLeft = new TalonFX(Constants.leftIntakeLiftId);
  TalonSRX intake = new TalonSRX(Constants.intakeId);
  TalonFX leftShooter = new TalonFX(Constants.leftShooterId);
  TalonFX rightShooter = new TalonFX(Constants.rightShooterId);
  DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(Constants.encoder));
  ColorSensorV3 noteChecker = new ColorSensorV3(Port.kOnboard);
  // variables
  boolean noteLoaded = false;
  boolean shooting = false;
  AbsoluteEncoder liftEncoder;
  PIDController controller = new PIDController(Constants.intakeP, 0, 0);

  public enum IntakeMode {
    normal,
    shooter,
    amp,
    halt
  };

  public IntakeMode mode = IntakeMode.normal;

  public Intake() {
    intakeLiftRight.setInverted(true);
  }

  public double getEncoder(){
    return encoder.getAbsolutePosition();
  }

  public boolean getNote(){
    return noteLoaded;
  }

  public void goToSetpoint(double setpoint){
    setpoint /= 360.0;
    double calculatedSpeed = controller.calculate(getEncoder(), setpoint);
    intakeLiftLeft.set(calculatedSpeed);
    intakeLiftRight.set(calculatedSpeed);
  }

  public void setShooter(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void intake(){
    double setpoint = Constants.intakeDegree;
    if (this.mode == IntakeMode.normal) {
      setpoint = Constants.intakeDegree;
      if (!noteLoaded){
        setIntakeMotor(false);
      }
      else {
        stopIntake();
      } 
    }
    else if (this.mode == IntakeMode.shooter){
      setpoint = Constants.shooterDegree;
    }
    else if (this.mode == IntakeMode.amp){
      setpoint = Constants.ampDegree;
      }
    goToSetpoint(setpoint);
  }

  public void setIntakeMode(IntakeMode mode) {
    this.mode = mode;
  }

  public void setIntakeMotor(boolean reverse){
    if(reverse){
      intake.set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    }
    else {
      intake.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    }
  }
  public void stopIntake(){
    intake.set(ControlMode.PercentOutput ,0);
  }

  @Override
  public void periodic() {
    if (noteChecker.getIR() > 100){
      noteLoaded = true;
    }
    else {
      noteLoaded = false;
    }
    SmartDashboard.putNumber("Encoder Value", getEncoder());
    SmartDashboard.putBoolean("Note Loaded", noteLoaded);
    SmartDashboard.putString("Intake Mode", mode.name());
  }
}
