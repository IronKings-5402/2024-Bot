// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // motors and devices
  TalonFX intakeLiftRight = new TalonFX(Constants.rightIntakeLiftId);
  TalonFX intakeLiftLeft = new TalonFX(Constants.leftIntakeLiftId);
  TalonSRX intake = new TalonSRX(Constants.intakeId);
  TalonFX leftShooter = new TalonFX(Constants.leftShooterId);
  TalonFX rightShooter = new TalonFX(Constants.rightShooterId);
  DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(Constants.encoder));
  DigitalInput noteChecker = new DigitalInput(1);
  // variables
  boolean noteLoaded = false;
  boolean intakeOn = false;
  AbsoluteEncoder liftEncoder;
  PIDController controller = new PIDController(Constants.intakeP, 0, Constants.intakeD);
  Swerve s_Swerve;
  double distance = 0;
  double calcSetpoint = 125;
  double currentSetpoint = Constants.shooterDegree;
  boolean manual = false;
  Timer timer = new Timer();
  public enum IntakeMode {
    normal,
    shooter,
    amp,
    halt
  };

  public IntakeMode mode = IntakeMode.shooter;


  public Intake() {
    intake.setNeutralMode(NeutralMode.Brake);
    intakeLiftLeft.setNeutralMode(NeutralModeValue.Brake);
    intakeLiftRight.setNeutralMode(NeutralModeValue.Brake);
    intakeLiftRight.setInverted(false);
    intakeLiftLeft.setInverted(true);
    leftShooter.setNeutralMode(NeutralModeValue.Brake);
    rightShooter.setNeutralMode(NeutralModeValue.Brake);
    SmartDashboard.putNumber("test setpoint", 125);
  }

  public double getEncoder(){
    return encoder.getAbsolutePosition()*360;
  }


  public void setRumble(boolean on){
    if (on){
      new Joystick(0).setRumble(RumbleType.kBothRumble, 1);
    }
    else{
      new Joystick(0).setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public boolean getNote(){
    return noteLoaded;
  }

  public void goToSetpoint(double setpoint){
    double calculatedSpeed = controller.calculate(getEncoder(), setpoint);
    calculatedSpeed = MathUtil.clamp(calculatedSpeed, -Constants.intakeMaxSpeed, Constants.intakeMaxSpeed);
    intakeLiftLeft.set(calculatedSpeed);
    intakeLiftRight.set(calculatedSpeed);
  }

  public void setShooter(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void manualLift(double speed){
    intakeLiftLeft.set(speed);
    intakeLiftRight.set(speed);
  }

  public void intake(){
    if (manual){
      if (intakeOn){
        setIntakeMotor(true);
      }
      else {
        stopIntake();
      }
      return;
    }
    double setpoint = currentSetpoint;
    if (this.mode == IntakeMode.normal) {
      setpoint = Constants.intakeDegree;
      if (intakeOn){
        if (!noteLoaded){
          setIntakeMotor(true);
        }
        else {
          setIntakeMode(IntakeMode.halt);
          stopIntake();
        }
      }
      else {
        stopIntake();
      }
    }

    else if (this.mode == IntakeMode.shooter){
      if (LimelightHelpers.getTV("limelight-april")){
        //setpoint = calcSetpoint;
        setpoint = calcSetpoint;
      }
      else if (currentSetpoint < 100 || currentSetpoint > 175){
        setpoint = Constants.shooterDegree;
      }
    }

    else if (this.mode == IntakeMode.amp){
      setpoint = Constants.ampDegree;
    }

    else if (this.mode == IntakeMode.halt){
      setpoint = Constants.haltDegree;
      stopIntake();
    }

    goToSetpoint(setpoint);
    currentSetpoint = setpoint;
  }

  public double getDistance(){
    return distance;
  }

  public void toggleIntake(BooleanSupplier followingNote){
    if (followingNote.getAsBoolean()){
      return;
    }
    intakeOn = !intakeOn;
  }

  public void toggleIntake(boolean value){
    intakeOn = value;
  }

  public void setIntakeMode(IntakeMode mode) {
    this.mode = mode;
  }

  public void toggleManual() {
    this.manual = !manual;
  }

  public void setIntakeMotor(boolean reverse){
    if(reverse){
      intake.set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    }
    else {
      intake.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    }
  }

  public void setIntakeMotor(boolean reverse, double speed){
    if(reverse){
      intake.set(ControlMode.PercentOutput, -speed);
    }
    else {
      intake.set(ControlMode.PercentOutput, speed);
    }
  }

  public void stopIntake(){
    intake.set(ControlMode.PercentOutput ,0);
  }

  public void setManual(boolean manual){
    this.manual = manual;
  }


  @Override
  public void periodic() {
    intake();
    if (!noteChecker.get()){
      noteLoaded = true;
    }
    else {
      //setRumble(false);
      noteLoaded = false;
    }

    // AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Optional<Pose3d> id3 = layout.getTagPose(3);
    // Translation2d id = id3.get().toPose2d().getTranslation();
    // Translation2d pose = LimelightHelpers.getBotPose3d_wpiBlue("limelight-april").toPose2d().getTranslation();
    // distance = pose.getDistance(id);
    
    distance = (57-16.5)/ Math.tan(Math.toRadians(LimelightHelpers.getTY("limelight-april")+29));
    //calcSetpoint = distance * 0.194 + 104;
    if (distance < 55){
      calcSetpoint = distance * 0.197 + 105.5;
    }
    else if (distance < 100){
      calcSetpoint = distance * 0.197 + 106;
    }
    else if (distance < 120 && distance >= 100) {
      calcSetpoint = distance * 0.215 + 105;
    }
    else if (distance < 130 && distance >= 120){
      calcSetpoint = distance * 0.215 + 103;
    }
    else {
      calcSetpoint = distance * 0.215 + 100;
    }
    //calcSetpoint = distance * 0.195 + 105;
    //calcSetpoint = distance *.845+29.2;
  }
}
