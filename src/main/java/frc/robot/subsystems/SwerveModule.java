// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private FlywheelSim driveSim;
  private FlywheelSim turnSim;

  private final Encoder driveEncoder;
  private final Encoder turnEncoder;

  private final PIDController drivePIDController = new PIDController(ModuleConstants.PIDp, ModuleConstants.PIDi, ModuleConstants.PIDd);
  private final PIDController turnPIDController = new PIDController(ModuleConstants.ProfiledPIDp, ModuleConstants.ProfiledPIDi, ModuleConstants.ProfiledPIDd);

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.DriveKs, ModuleConstants.DriveKv);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(ModuleConstants.TurnKs, ModuleConstants.TurnKv);

  private EncoderSim driveEncoderSim;
  private EncoderSim turnEncoderSim;

  public double turnOutput;
  public double driveOutput;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int drivingEncoderChannelA, int drivingEncoderChannelB,
  int turningEncoderChannelA, int turningEncoderChannelB) {
    driveMotor = new TalonFX(0);
    turnMotor = new TalonFX(1);
    //turnMotor 
    
    driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 0, 0);
    turnSim = new FlywheelSim(DCMotor.getFalcon500(1), 0, 0);

    driveEncoder = new Encoder(drivingEncoderChannelA, drivingEncoderChannelB);
    turnEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    
    driveEncoderSim = new EncoderSim(driveEncoder);
    turnEncoderSim = new EncoderSim(turnEncoder);

    //driveEncoder.setDistancePerPulse(SOME CALCULATION [I NEED WHEEL DIAMETER]);
    //turnEncoder.setDistancePerPulse(SOMECALUCLATION [I NEED WHEEL DIAMETER]);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turnEncoder.getDistance()));
  }

  public SwerveModuleState getPosition() {
    return new SwerveModuleState(driveEncoder.getDistance(), new Rotation2d(turnEncoder.getDistance()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getDistance()));
    
    driveOutput = drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    double driveFeedForwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

    turnOutput = turnPIDController.calculate(turnEncoder.getRate(), state.angle.getRadians());

    double turnFeedForwardOutput = turnFeedforward.calculate(turnPIDController.getSetpoint());

    driveMotor.set(TalonFXControlMode.Velocity, driveOutput/DriveConstants.kMaxSpeed*RobotController.getBatteryVoltage()+driveFeedForwardOutput);
    turnMotor.set(TalonFXControlMode.Velocity, turnOutput/ModuleConstants.kModuleMaxAngularVelocity*RobotController.getBatteryVoltage()+turnFeedForwardOutput);
  }

  /*public CANSparkMax getDriveMotor(){
    return driveMotor;
  }

  public CANSparkMax getTurnMotor(){
    return turnMotor;
  }

  public double getDriveSpeed(){
    return driveMotor.get();
  }

  public double getTurnSpeed(){
    return driveMotor.get();
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    setDesiredState(getPosition());
    System.out.println("Working");
    //REVPhysicsSim.getInstance().run();
    //driveSim.setInputVoltage(getDriveSpeed());
    // This method will be called once per scheduler run
    
    //turnMotor.set(turnOutput / ModuleConstants.kModuleMaxAngularVelocity * RobotController.getBatteryVoltage());

    //TalonFXControlMode

    //driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, frameTime);
  }
}
