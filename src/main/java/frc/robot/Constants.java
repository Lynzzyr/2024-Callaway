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

  public static final class kOperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class kCANID {
    public static final int idMotorFL = 10;
    public static final int idMotorFR = 11;
    public static final int idMotorBL = 12;
    public static final int idMotorBR = 13;

    public static final int idPigeon2 = 5;
  }

  public static final class kDrivetrain {
    public static final double kWheelDiameter = 0.1; // meters
    public static final double kWheelCircumference = Math.PI * kWheelDiameter;

    public static final int kCurrentLimit = 30;

    public static final double kCLRampRate = 0.5;
  }
}
