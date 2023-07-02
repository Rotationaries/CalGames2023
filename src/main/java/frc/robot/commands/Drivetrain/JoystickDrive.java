// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  XboxController controller;
  Drivetrain drive;
  boolean fieldRelative;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public JoystickDrive(XboxController controller, Drivetrain drive, boolean fieldRelative) {
    this.controller = controller;
    this.drive=drive;
    this.fieldRelative = fieldRelative;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Inverted X speed / Forwards and backwards

   // final double xSpeed = MathUtil.applyDeadband(controller.getLeftY(), 0.02);

    final double xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
            * DriveConstants.kMaxSpeed;

   // final double ySpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.02);

    // Inverted Y speed / Strafe speed
    final double ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * DriveConstants.kMaxSpeed;

   // final double rot = controller.getRightX();

    // Inverted angular rotation value
    final double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * DriveConstants.kMaxAngularSpeed;

      SmartDashboard.putNumber("Rottation", rot);
    //System.out.println("current rotation: " + rot);
    //System.out.println("Drive Speed (x): " + xSpeed + "Drive Speed (y): " + ySpeed + "Rotation Speed: " + rot );

    drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    //System.out.println("Joystick is controlling robot!");
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
