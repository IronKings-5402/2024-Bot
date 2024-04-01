// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Climber extends SubsystemBase {
  CANSparkMax leftClimber = new CANSparkMax(44, MotorType.kBrushless);
  CANSparkMax rightClimber = new CANSparkMax(45, MotorType.kBrushless);
  
  /** Creates a new Climber. */
  public Climber() {
  }
  public enum ClimberSide {left, right};
  public void climberSet(ClimberSide side, boolean reverse){
    double speed = Constants.climberSpeed;
    if (reverse){
      speed = -speed;
    } 
    if (side == ClimberSide.left){
      leftClimber.set(speed);
    }
    else if(side == ClimberSide.right){
      rightClimber.set(speed);
    }
  }
  public void climberStop(ClimberSide side){
    if (side == ClimberSide.left){
      leftClimber.set(0);
    }
    else if (side == ClimberSide.right){
      rightClimber.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
