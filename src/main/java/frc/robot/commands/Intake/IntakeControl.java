// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeControl extends CommandBase {
  XboxController controller;
  Intake intake;
  

  /** Creates a new IntakeControl. */
  public IntakeControl(XboxController controller, Intake intake) {
    this.controller = controller;
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(controller.getAButton()){
      intake.intake(IntakeConstants.intakeIndexSpeed, IntakeConstants.intakeFlywheelSpeed);
    }else{
      intake.stop();
    }

    if(controller.getYButton()){
      intake.outtake(IntakeConstants.outtakeIndexSpeed, IntakeConstants.outtakeFlywheelSpeed);
    }else{
      intake.stop();
    }

    if(controller.getRightBumper()){
      intake.startFlywheel(IntakeConstants.launchFlywheelSpeed);
    }

    if(controller.getLeftBumper()){
      intake.startIndexer(IntakeConstants.launchIndexSpeed);
    }

    SmartDashboard.putNumber("Intake Flywheel Speed", intake.getFlywheelSpeed());
    SmartDashboard.putNumber("Intake Indexer Speed", intake.getIndexSpeed());
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
