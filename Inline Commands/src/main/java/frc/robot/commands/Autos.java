// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANDriveSubsystem;


public final class Autos {
 private CANDriveSubsystem driveSubsystem = CANDriveSubsystem.get();
  // Example autonomous command which drives forward for 1 second.
  private double speed = .5588;
  // change the numerator (its the distance you want)
  private double motorSpeed = .5;
  private Command leaveBarge = driveSubsystem.tankDrive(driveSubsystem, () -> -motorSpeed, () -> motorSpeed).withTimeout(2);
  private Command diagonalToTapeOfReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(3.77/speed);
  private Command tapeToReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(.365/speed);
  private Command backingUpFromReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(1.939/speed);
  private Command backUpToCoral= driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(6.334/speed);
  private Command distanceFromAngleToCoral = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(.792/speed);
  private Command coralToOneThirdReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(.792/speed);
  private Command coralToTwoThirdReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(2.974/speed);

  
  
  
  public Command justLeave() {
    System.out.println("int the auto");
    return new AngleCommands(90);
    // return Commands.run(() -> System.out.println("justleave"));
    // return diagonalToTapeOfReef.withTimeout(3.77).until(()->false);
  }
  
  public Command scoreLoneOnce() {
    System.out.println("Score L1 ONCE");
    return Commands.runOnce(()-> System.out.println("lsdjf"));
  }
}
// public final class Autos {
    
//     public static final Command testAuto (CANDriveSubsystem driveSubsystem) {
//         return driveSubsystem.tankDrive(driveSubsystem, () -> .5, () -> .5).withTimeout(2);
//     }
// }