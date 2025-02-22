// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;
import java.util.function.DoubleSupplier;

import javax.naming.NameNotFoundException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;

  private final DifferentialDrive drive;
  private static final CANDriveSubsystem instance = new CANDriveSubsystem();

  private final AHRS navx;

  // private final DifferentialDriveOdometry m_Odometry;
  // private DifferentialDriveKinematics differentialDriveKinematics;
  // private DifferentialDriveWheelPositions encoders;

  private CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new WPI_TalonSRX(2);
    leftFollower = new WPI_TalonSRX(1);
    rightLeader = new WPI_TalonSRX(4);
    rightFollower = new WPI_TalonSRX(3);
    navx = new AHRS(SPI.Port.kMXP);  // SPI connection for NavX

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // differentialDriveKinematics = new DifferentialDriveKinematics(0.251);

    // encoders = new DifferentialDriveWheelPositions(null, null);

    // m_Odometry = new
    // DifferentialDriveOdometry(navX.getRotation2d(),null,null,getPose());
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(12);
    // config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    // Remove following, then apply config to right leader
    rightLeader.setInverted(true);
    leftLeader.setInverted(false);
    rightFollower.setInverted(true);

    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward

    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    // RobotConfig config;
    // try{
    // config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    // // Handle exception as needed
    // e.printStackTrace();
    // return;
    // }

    // // Configure AutoBuilder last
    // AutoBuilder.configure(
    // this::getPose, // Robot pose supplier
    // this::resetPose, // Method to reset odometry (will be called if your auto has
    // a starting pose)
    // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will
    // drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs
    // individual module feedforwards
    // new PPLTVController(0.02), // PPLTVController is the built in path following
    // controller for differential drive trains
    // config, // The robot configuration
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
    // },
    // this // Reference to this subsystem to set requirements
    // );

    // private Pose2d getPose() {
    // return new Pose2d(new Translation2d(), navX.getRotation2d());
    // }

    // private void resetPose(Pose2d pose2d) {
    // m_Odometry.resetPosition(navX.getRotation2d(),encoders , pose2d);
    // }

    // private ChassisSpeeds getRobotRelativeSpeeds() {
    // return null;
    // }

    // private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {

  }

  @Override
  public void periodic() {
  }

  public static CANDriveSubsystem get() {
    return instance;
  }

  // Command to drive the robot with joystick inputs
  public Command tankDrive(
      CANDriveSubsystem driveSubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    return Commands.run(
        () -> {
          // System.out.println("Tank Drive Auto");
          drive.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
        }, driveSubsystem);

  }
  public double getYaw() {
    return navx.getYaw();
}

// Reset the NavX's yaw to zero
public void resetNavX() {
    navx.reset();
}
  // public Command encoderValue() {
  // return Commands.runOnce(() -> {
  // leftFollower.getSelectedSensorPosition();
  // System.out.println(leftFollower.getSelectedSensorPosition());
  // });

}
