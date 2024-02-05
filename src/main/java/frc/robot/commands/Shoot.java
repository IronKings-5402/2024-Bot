// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  Intake s_Intake;
  Timer timer = new Timer();
  boolean end = false;
  public Shoot(Intake s_Intake) {
    this.s_Intake = s_Intake;
    addRequirements(this.s_Intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!s_Intake.getNote() || !(s_Intake.mode == IntakeMode.shooter) || !(s_Intake.mode == IntakeMode.amp) ){
      end = true;
    }

    if (s_Intake.mode == IntakeMode.shooter){
      s_Intake.setShooter(Constants.shooterSpeed);
      if(timer.get() > 1.0){
        s_Intake.setIntakeMotor(false);
      }
      
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    s_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}