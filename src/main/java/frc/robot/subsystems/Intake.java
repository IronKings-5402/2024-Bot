// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
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
  PIDController controller = new PIDController(Constants.intakeP, 0, 0);
  Swerve s_Swerve;
  double distance = 0;
  double calcSetpoint = 125;
  double currentSetpoint = Constants.shooterDegree;
  boolean manual = false;
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

  public boolean getNote(){
    return noteLoaded;
  }

  public void goToSetpoint(double setpoint){
    double calculatedSpeed = controller.calculate(getEncoder(), setpoint);
    calculatedSpeed = MathUtil.clamp(calculatedSpeed, -.35, .35);
    intakeLiftLeft.set(calculatedSpeed);
    intakeLiftRight.set(calculatedSpeed);
  }

  public void setShooter(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void intake(){
    if (manual){
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

  public void toggleIntake(){
    intakeOn = !intakeOn;
  }

  public void toggleIntake(boolean value){
    intakeOn = value;
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
      noteLoaded = false;
    }

    // AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Optional<Pose3d> id3 = layout.getTagPose(3);
    // Translation2d id = id3.get().toPose2d().getTranslation();
    // Translation2d pose = LimelightHelpers.getBotPose3d_wpiBlue("limelight-april").toPose2d().getTranslation();
    // distance = pose.getDistance(id);
    
    distance = (57-16.5)/ Math.tan(Math.toRadians(LimelightHelpers.getTY("limelight-april")+29));
    SmartDashboard.putNumber("Distance", distance);
    if (LimelightHelpers.getTV("limelight-april")){
      SmartDashboard.putBoolean("Valid Apriltag target", true);
      SmartDashboard.putNumber("FID", LimelightHelpers.getFiducialID("limelight-april"));
    }
    else {
      SmartDashboard.putBoolean("Valid Apriltag target", false);
      SmartDashboard.putNumber("FID", 404);
    }
    calcSetpoint = distance * 0.191 + 104;
    SmartDashboard.putBoolean("AI target", LimelightHelpers.getTV("limelight-ai"));
    SmartDashboard.putBoolean("Note Loaded", noteLoaded);
    SmartDashboard.putString("Intake Mode", mode.name());
  }
}
