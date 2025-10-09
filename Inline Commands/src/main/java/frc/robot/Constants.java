// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID_1 = 1;
    public static final int LEFT_FOLLOWER_ID_2 = 2;
    public static final int RIGHT_LEADER_ID_3 = 3;
    public static final int RIGHT_FOLLOWER_ID_4 = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double kMaxSpeed = 5.2;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);

    public static final double kMotorKs = 0.176;
    public static final double kMotorKv = 0.177;
    public static final double kMotorReduction = 8.45;
    public static final double kRampRateSeconds = 1.0;
    public static final int kCurrentLimit = 60;

  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 10;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double Roller_Min_Ticks = 0;
    public static final double Roller_Max_Ticks = 1000;
    public static final double ROLLER_EJECT_VALUE = 0.25;

  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static final class algaeActuatorConstants {
    public static final double Min_Tics = 0;
    public static final double Max_Tics = -20;
  }
}
