// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class FollowNote extends Command {
  Swerve s_Swerve;
  double offset;
  TeleopSwerve teleCommand;
  PIDController controller;
  DoubleSupplier translationValue;
  /** Creates a new FollowNote. */
  public FollowNote(Swerve s_Swerve, DoubleSupplier translationValue) {
    controller = new PIDController(Constants.shooterP, 0, 0);
    offset = Constants.pigeonOffset;
    this.s_Swerve = s_Swerve;
    this.translationValue = translationValue;
    addRequirements(this.s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("limelight-ai")) {
      return;
    }
    controller.setTolerance(offset);
    double rotationVal = controller.calculate(LimelightHelpers.getTX("limelight-ai"), 0);
    s_Swerve.drive(
            new Translation2d(translationValue.getAsDouble(), 0), 
            rotationVal,
            false, 
            true
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
