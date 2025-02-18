package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax algeaIntakeRoller;
    private final SparkMax algeaIntakeActuator;
    private static final AlgaeIntake instance = new AlgaeIntake();
    
    private AlgaeIntake() {
        // TODO find actual motor IDs
        algeaIntakeRoller = new SparkMax(0, MotorType.kBrushless);
        algeaIntakeActuator = new SparkMax(0, MotorType.kBrushless);
    }
    public static AlgaeIntake get() {
                return instance;
    }
    public Command algeaRoller() {
        return Commands.run(() -> System.out.println("working roller"));
    }

    public Command algeaActuator() {
        return Commands.run(() -> System.out.println("working actuator"));
    }
}
