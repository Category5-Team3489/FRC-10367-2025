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
  private Command diagonalToTapeOfReef = driveSubsystem.tankDrive(driveSubsystem, () -> motorSpeed, () -> motorSpeed).withTimeout(3);

  public Command autos() {
    System.out.println("int the auto");
    return diagonalToTapeOfReef.repeatedly();
    // return diagonalToTapeOfReef.withTimeout(3.77).until(()->false);
  }
}
// public final class Autos {
    
//     public static final Command testAuto (CANDriveSubsystem driveSubsytem) {
//         return driveSubsytem.tankDrive(driveSubsytem, () -> .5, () -> .5).withTimeout(2);
//     }
// }