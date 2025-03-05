// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

import java.util.PrimitiveIterator;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;




/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private double targetTics = 500;
  private int ticsPerRotation = 4096;
  private final int gearRatio = 1;
  private static final CANRollerSubsystem instance = new CANRollerSubsystem();
  private double forward = .25;


  // private CANSparkMax backUpMotor;
  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    encoder = rollerMotor.getEncoder();
    pidController = rollerMotor.getClosedLoopController();
    
   

   
    


    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // rollerMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    // SparkMaxConfig rollerConfig = new SparkMaxConfig();
    // rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    // rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    // rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  // Command to run the roller with joystick inputs


  public static CANRollerSubsystem get() {
    return instance;
}

public Command runRoller(
  CANRollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
return Commands.run(
    () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
}
public Command rollerTest() {
  System.out.println("rollerTest command triggered"); 
  return Commands.run(() -> {
    System.out.println("rollertest should run"); 
    rollerMotor.set(forward);
      System.out.println("Motor is running at 0.25"); 
  });
}


  public Command littleRoller() {
    return Commands.run(() -> rollerMotor.set(.12),this);
  }

  public Command swiftness() {
    return Commands.run(() -> rollerMotor.set(.5), this);
   
  }

  public Command rollerReverse() {
    return Commands.run(() -> rollerMotor.set(-.2), this);
  }

  // public Command rollerFraction() {

  //   return Commands.runOnce(() -> {
  //     encoder.setPosition(0);
  //     double newPosition = encoder.getPosition() + 1000;
  //     System.out.println(newPosition);
  //     rollerMotor.set(.3);
  //     encoder.setPosition(newPosition);
  //   }, this); }

 public Command setTargetTics(double tics) {
        return Commands.runOnce(() -> {targetTics = MathUtil.clamp(tics, RollerConstants.Roller_Min_Ticks, RollerConstants.Roller_Max_Ticks);
    },this);
    }
  
  public Command rollerFun() {
    return Commands.runOnce(() -> {
      double targetRotation = (targetTics * gearRatio) / ticsPerRotation;
      System.out.println(targetRotation);
      pidController.setReference(targetRotation, ControlType.kPosition, ClosedLoopSlot.kSlot1, 12);
      encoder.setPosition(1);
      // rollerMotor.

    });
  }

  public Command rollerUpdate(double newTics) {
    targetTics = MathUtil.clamp(newTics, RollerConstants.Roller_Min_Ticks, RollerConstants.Roller_Max_Ticks);
    return rollerFun();
  }
}
