// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // motors and devices
  CANSparkMax intakeLiftRight = new CANSparkMax(Constants.rightIntakeId, MotorType.kBrushless);
  CANSparkMax intakeLiftLeft = new CANSparkMax(Constants.leftIntakeId, MotorType.kBrushless);
  TalonFX intake = new TalonFX(Constants.intakeId);
  TalonFX leftShooter = new TalonFX(Constants.leftShooterId);
  TalonFX rightShooter = new TalonFX(Constants.rightShooterId);

  ColorSensorV3 noteChecker = new ColorSensorV3(Port.kOnboard);
  SparkPIDController liftSparkPIDController;
  // variables
  boolean noteLoaded = false;
  boolean shooting = false;
  AbsoluteEncoder liftEncoder;

  public enum IntakeMode {
    normal,
    shooter,
    amp,
    halt
  };

  public IntakeMode mode = IntakeMode.normal;

  public Intake() {
    intakeLiftRight.setInverted(true);

    liftSparkPIDController = intakeLiftRight.getPIDController();
    liftEncoder = intakeLiftRight.getAbsoluteEncoder(Type.kDutyCycle);
    liftSparkPIDController.setP(Constants.intakeP);
    liftSparkPIDController.setFeedbackDevice(liftEncoder);

    intakeLiftRight.burnFlash();
  }

  public double getEncoder(){
    return intakeLiftRight.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
  }

  public boolean getNote(){
    return noteLoaded;
  }

  public void goToSetpoint(double setpoint){
    setpoint /= 360.0;
    
    liftSparkPIDController.setReference(setpoint, ControlType.kDutyCycle);
    intakeLiftLeft.follow(intakeLiftRight);
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
        intake.set(Constants.intakeSpeed);
      }
      else {
        intake.set(0);
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
      intake.set(-Constants.intakeSpeed);
    }
    else {
      intake.set(Constants.intakeSpeed);
    }
  }
  public void stopIntake(){
    intake.set(0);
  }

  @Override
  public void periodic() {
    if (noteChecker.getIR() > 100){
      noteLoaded = true;
    }
    else {
      noteLoaded = false;
    }
  }
}
