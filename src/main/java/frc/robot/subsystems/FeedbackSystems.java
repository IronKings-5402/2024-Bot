// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake.IntakeMode;

public class FeedbackSystems extends SubsystemBase {
  /** Creates a new FeedbackSystems. */
  Spark blinkin = new Spark(2);
  Joystick joystick;
  boolean seesTag = false;
  boolean hasNote = false;
  BooleanSupplier noteLoaded;
  DoubleSupplier distance;
  Supplier<IntakeMode> mode;
  boolean tripped;
  Timer timer = new Timer();
  public FeedbackSystems(Joystick joystick, BooleanSupplier noteLoaded, Supplier<IntakeMode> mode, DoubleSupplier distance) {
    this.joystick = joystick;
    this.noteLoaded = noteLoaded;
    this.mode = mode;
    this.distance = distance;
    tripped = noteLoaded.getAsBoolean();
  }

  public void setGreen(){
    blinkin.set(.77);
  }
  public void setRainbow(){
    blinkin.set(-.97);
  }

  public void setRumble(boolean on){
    if (on){
      joystick.setRumble(RumbleType.kBothRumble, .4);
    }
    else {
      joystick.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public void periodic() {
    if (LimelightHelpers.getTV("limelight-april")){
      SmartDashboard.putBoolean("Valid Apriltag target", true);
      SmartDashboard.putNumber("FID", LimelightHelpers.getFiducialID("limelight-april"));
      setGreen();
    }
    else {
      SmartDashboard.putBoolean("Valid Apriltag target", false);
      SmartDashboard.putNumber("FID", 404);
      setRainbow();
    }
    SmartDashboard.putBoolean("AI target", LimelightHelpers.getTV("limelight-ai"));
    SmartDashboard.putBoolean("Note Loaded", noteLoaded.getAsBoolean());
    SmartDashboard.putString("Intake Mode", mode.get().name());
    SmartDashboard.putNumber("Distance", distance.getAsDouble());
    if (mode.get() == IntakeMode.halt){
      SmartDashboard.putBoolean("Can Drive Under", true);
    }
    else {
      SmartDashboard.putBoolean("Can Drive Under", false);
    }

    if (DriverStation.isEnabled() && !DriverStation.isAutonomous()){
      if (noteLoaded.getAsBoolean()){
        if (!tripped){
          timer.start();
          setRumble(true);
          if (timer.get() > 1.2){
            tripped = true;
            setRumble(false);
            }
        }
        else {
          tripped = false;
        }
      }
      else{
        timer.stop();
        timer.reset();
        tripped = false;
        setRumble(false);
      }
    }
    else {
      setRumble(false);
    }

  }
}
