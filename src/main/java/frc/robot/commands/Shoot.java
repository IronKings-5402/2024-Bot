// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeMode;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  Intake s_Intake;
  Swerve s_Swerve;
  Timer timer = new Timer();
  boolean end = false;
  DoubleSupplier translation;
  DoubleSupplier strafe;
  PIDController controller = new PIDController(Constants.shooterP, 0, 0);
  public Shoot(Intake s_Intake, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.s_Swerve = s_Swerve;
    this.s_Intake = s_Intake;
    this.translation = translationSup;
    this.strafe = strafeSup;
    addRequirements(this.s_Intake, this.s_Swerve);
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
    // if there is not a note or the IntakeMode is not shooter 
    double shootSpeed = 0;
    if (s_Intake.mode == IntakeMode.amp){
      shootSpeed = Constants.ampSpeed;
    }
    else {
      shootSpeed = Constants.shooterSpeed;
    }

    s_Intake.setShooter(shootSpeed);

    if (DriverStation.isAutonomous()){
      if (timer.get()> 1){
      end = true;
      }
      else if (timer.get() > .5)
      s_Intake.setIntakeMotor(true);
    }

    else {
      if (timer.get()> 2.5){
        end = true;
      }

      else if(timer.get() > 1.5){
        s_Intake.setIntakeMotor(true);
      }
    }

    
    // timeout  
    
    
    double rotationVal = 0;
    if (LimelightHelpers.getTV("limelight-april")){
      rotationVal = controller.calculate(LimelightHelpers.getTX("limelight-april"), 0);
    }
    double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.stickDeadband);
    s_Swerve.drive(
    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
    rotationVal, 
    true, 
    true
    );
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    //s_Intake.toggleIntake(false);
    if (!DriverStation.isAutonomous()){
          s_Intake.setShooter(0);
    }
    s_Intake.stopIntake();
    end = false;
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
