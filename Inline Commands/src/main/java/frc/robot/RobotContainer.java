// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.OptionalInt;

import javax.xml.transform.sax.TemplatesHandler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Autos autos = new Autos();
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = CANDriveSubsystem.get();
  private final CANRollerSubsystem rollerSubsystem = CANRollerSubsystem.get();
  private final AlgaeIntake algaeIntake= AlgaeIntake.get();
  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Optional<Alliance> alliance = DriverStation.getAlliance();
    OptionalInt location = DriverStation.getLocation();

    // if (alliance.isPresent() && alliance.get() == Alliance.Red) {

    // } else { //Blue

    // }

    if (location.isPresent() && location.getAsInt() == 2) {
      autoChooser.setDefaultOption("Autonomous",Autos.scoreLoneOnce(driveSubsystem, 2.234));
    } else {
      autoChooser.setDefaultOption("Autonomous",Autos.scoreLoneOnce(driveSubsystem, 3.651));
    }

    

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    // autoChooser.setDefaultOption("Autonomous",autos.justLeave());

    // autoChooser.setDefaultOption("Autonomous", Commands.parallel(
    //   Commands.runOnce(() -> driveSubsystem.move(.5588,.5588), driveSubsystem), 
    //   Commands.waitSeconds(1))
    //   .andThen(
    //     Commands.runOnce(() -> driveSubsystem.move(0.0,0.0), driveSubsystem)
    //   ));

    // autoChooser.addOption("Turn Left",autos.justLeave());
    // autoChooser.addOption("Cat5 123", autos.autos());


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "runRoller" command from the factory with a fixed
    // value ejecting the gamepiece while the button is held
    operatorController.y()
    // .onTrue(rollerSubsystem.rollerUpdate(1000));
    .onTrue(rollerSubsystem.runRollerOnce(rollerSubsystem, () -> .12, () -> 0).andThen(Commands.waitSeconds(1).andThen(rollerSubsystem.runRollerOnce(rollerSubsystem, () -> .12, () -> .12))));

    operatorController.a()
       .onTrue(algaeIntake.algaeActuator());
        
    operatorController.x()
    .whileTrue(rollerSubsystem.runRoller(rollerSubsystem, () -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0));
    

    
    operatorController.b() 
    .whileTrue(rollerSubsystem.runRoller(rollerSubsystem, () -> -.20, () -> 0));
    
    // operatorController.rightBumper()
    //   .onTrue(driveSubsystem.encoderValue());


    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(
        driveSubsystem.tankDrive(driveSubsystem,() -> -driverController.getLeftY(), () -> -driverController.getRightY()));

    // Set the default command for the roller subsystem to the command from the
    // factory with the values provided by the triggers on the operator controller

    // rollerSubsystem.setDefaultCommand(
    //     rollerSubsystem.runRoller(
    //         rollerSubsystem,
    //         () -> operatorController.getRightTriggerAxis(),
    //         () -> operatorController.getLeftTriggerAxis()));

    rollerSubsystem.setDefaultCommand(
        rollerSubsystem.runRoller(
            rollerSubsystem,
            () -> 0,
            () -> 0));

    operatorController.rightTrigger()
        .whileTrue(algaeIntake.algaeRoller());
    
    operatorController.leftTrigger()
        .whileTrue(algaeIntake.algaeNegativeRoller());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
