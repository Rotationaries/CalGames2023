// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    //FL-Front Left, DM-Driving Motor, TM-Turning Motor, DE-Driving Encoder, TE-Turning Encoder
    public static final int FLDMChannel = 1;
    public static final int FLTMChannel = 2;
    public static final int FLTEChannelA = 2;
    public static final int FLTEChannelB = 3;

    public static final int FRDMChannel = 3;
    public static final int FRTMChannel = 4;
    public static final int FRTEChannelA = 6;
    public static final int FRTEChannelB = 7;

    public static final int BLDMChannel = 5;
    public static final int BLTMChannel = 6;
    public static final int BLTEChannelA = 10;
    public static final int BLTEChannelB = 11;

    public static final int BRDMChannel = 7;
    public static final int BRTMChannel = 8;
    public static final int BRTEChannelA = 14;
    public static final int BRTEChannelB = 15;
  }
  
  public static class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public static final double PIDp = 1;
    public static final double PIDi = 0;
    public static final double PIDd = 0;

    public static final double ProfiledPIDp = 1;
    public static final double ProfiledPIDi = 0;
    public static final double ProfiledPIDd = 0;

    public static final double DriveKs = 1;
    public static final double DriveKv = 3;

    public static final double TurnKs = 1;
    public static final double TurnKv = 0.5;
  }
}
