package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANDriveSubsystem;

public class AngleCommands extends Command {

    private final CANDriveSubsystem driveSubsystem;
    private final double targetAngle;
    // private final PIDController pidController;

    // Constructor takes a delta angle to rotate
    public AngleCommands(double deltaAngle) {
        System.out.println("Start of constructor");

        this.driveSubsystem = CANDriveSubsystem.get();

        // Calculate the target angle relative to the current angle
        double currentAngle = driveSubsystem.getYaw(); // Get the current angle from NavX
        System.out.println("after yaw");
        this.targetAngle = currentAngle + deltaAngle; // Set the target angle by adding deltaAngle

        // Set up the PID controller
        // pidController = new PIDController(0.03, 0.0, 0.0); // P, I, D values (tune these)
        // pidController.setTolerance(2.0); // Set tolerance (in degrees) for when we should stop
        // pidController.setSetpoint(targetAngle); // Set the target angle
        addRequirements(driveSubsystem); // Register the subsystem
    }

    @Override
    public void initialize() {
        // Optionally reset the NavX before starting

        driveSubsystem.resetNavX();
        System.out.println("after navx reset");
    }

    @Override
    public void execute() {
        System.out.println("execute");

        // Get the current angle from NavX
        double targetAngle = driveSubsystem.getYaw();

        // Get the PID output (this will adjust motor speeds based on the current angle)
        // double pidOutput = pidController.calculate(targetAngle);

        // System.out.println(pidOutput);

        // Apply the PID output to control the robot
        // If pidOutput is positive, turn counterclockwise; if negative, turn clockwise
        // Right motor gets pidOutput
        // driveSubsystem.tankDrive(driveSubsystem, () -> -pidOutput, () -> pidOutput).andThen(Commands.waitSeconds(1)).schedule(); // Left motor gets -pidOutput

        driveSubsystem.move(-2.7, 2.7);
    }
    
    @Override
    public boolean isFinished() {
        System.out.println("pid point");
        // System.out.println(pidController.atSetpoint());
        System.out.println(driveSubsystem.getYaw());
        // Command finishes when we're within tolerance of the target angle
        // return pidController.atSetpoint();
        // return driveSubsystem.getYaw() == targetAngle;
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("beginning of end");
        // Stop the motors when done
        // driveSubsystem.tankDrive(driveSubsystem, () -> 0.0, () -> 0.0).schedule();
        driveSubsystem.move(0.0,0.0);
    }
}
