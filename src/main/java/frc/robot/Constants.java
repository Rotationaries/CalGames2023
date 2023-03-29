// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

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
    public static final int FLDEChannelA = 0;
    public static final int FLDEChannelB = 1;
    public static final int FLTEChannelA = 2;
    public static final int FLTEChannelB = 3;

    public static final int FRDMChannel = 3;
    public static final int FRTMChannel = 4;
    public static final int FRDEChannelA = 4;
    public static final int FRDEChannelB = 5;
    public static final int FRTEChannelA = 6;
    public static final int FRTEChannelB = 7;

    public static final int BLDMChannel = 5;
    public static final int BLTMChannel = 6;
    public static final int BLDEChannelA = 8;
    public static final int BLDEChannelB = 9;
    public static final int BLTEChannelA = 10;
    public static final int BLTEChannelB = 11;

    public static final int BRDMChannel = 7;
    public static final int BRTMChannel = 8;
    public static final int BRDEChannelA = 12;
    public static final int BRDEChannelB = 13;
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
