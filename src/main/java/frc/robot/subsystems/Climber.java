// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  TalonFX leftClimber = new TalonFX(Constants.leftClimberId);
  TalonFX rightClimber = new TalonFX(Constants.rightClimberId);
  /** Creates a new Climber. */
  public Climber() {
    this.rightClimber.setInverted(true);
  }

  public void climberUp(){
    leftClimber.set(.5);
    rightClimber.set(.5);
  }

  public void climberDown(){
    leftClimber.set(-.5);
    rightClimber.set(-.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
