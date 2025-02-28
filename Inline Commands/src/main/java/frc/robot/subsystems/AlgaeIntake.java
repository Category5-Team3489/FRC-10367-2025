package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeActuatorConstants;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax algeaIntakeRoller;
    private final SparkMax algeaIntakeActuator;
    private static final AlgaeIntake instance = new AlgaeIntake();
    private double targetTics = 0;
    private int ticsPerRotation = 4096;
    private final SparkClosedLoopController pidController;

    //TODO find gearRatio
    private final int gearRatio = 1;
    private boolean min = true;
    
    private AlgaeIntake() {
        // TODO find actual motor IDs
        algeaIntakeRoller = new SparkMax(1, MotorType.kBrushless);
        algeaIntakeActuator = new SparkMax(2, MotorType.kBrushless);

        pidController = algeaIntakeActuator.getClosedLoopController();
    }
    public static AlgaeIntake get() {
                return instance;
    }
    public Command algaeRoller() {
        return Commands.run(()->{
        System.out.println("working roller");
        algeaIntakeRoller.set(.5);}, this);
    }
    public Command algaeNegativeRoller() {
        return Commands.run(()->{
        System.out.println("working negative roller");
        algeaIntakeRoller.set(-.5);}, this);
    }
    //TODO ajust the tic values
    public Command setTargetTics(double tics) {
        return Commands.run(() -> targetTics = MathUtil.clamp(tics, algaeActuatorConstants.Max_Tics, algaeActuatorConstants.Min_Tics),this);
    }


    //TODO check the motor to see if needed to be inverted
    public Command algaeActuator() {
        return Commands.runOnce( () -> {
            if (min) {
                setTargetTics( algaeActuatorConstants.Max_Tics);
                min = false;
            }
            else if (!min) {
                setTargetTics( algaeActuatorConstants.Min_Tics);
                min = true;
            }
        });
    }
    //TODO find gearRatio
    private void setAngle() {
        double targetRotation = (targetTics * gearRatio) / ticsPerRotation;
        pidController.setReference(targetRotation, ControlType.kPosition, ClosedLoopSlot.kSlot0);}
    
    // public Command algaeActuator() {
    //     return Commands.run(() ->{ System.out.println("working actuator");
    //     },this);
    // }
}
