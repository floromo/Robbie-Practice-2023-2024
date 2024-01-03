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

  public class DrivebaseConstants {  
    public static final int kCountsPerRev = 42;
    public static final int kStallLimit = 40;
    public static final int kFreeLimit = 40;
    public static final int kWheelDiameterInches = 6;
  
  }

   public class MotorIDConstants {   
    public static final int kLeftSlave = 1;
    public static final int kLeftMaster = 2;
    public static final int kRightSlave = 3;
    public static final int kRightMaster = 4;
  }

  public class GearRatioConstants {
    public static final int kGearRatio = 4;
  }

  public class PIDControllerConstants {
    public static final double Kp = 0;
    public static final double Ki = 0;
    public static final double Kd = 0;
  }

  public class WristConstants {
    public static final int kLeftMotor = 1;
    public static final int kRightMotor = 2;
    public static final int kFF = 100;
    public static final int kP = 100;
    public static final int kD = 100;
    public static final double kWRIST_GEAR_RATIO = 5000;
  }
}
 