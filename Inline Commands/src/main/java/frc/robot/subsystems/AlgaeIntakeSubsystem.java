package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax algeaIntakeRoller;
    private final SparkMax algeaIntakeActuator;
    private static final AlgaeIntakeSubsystem instance = new AlgaeIntakeSubsystem();
    private double targetTics = 0;
    private int ticsPerRotation = 4096;
    private final SparkClosedLoopController pidController;

    // TODO find gearRatio
    private final int gearRatio = 1;
    private boolean min = true;

    private AlgaeIntakeSubsystem() {
        // TODO find actual motor IDs
        algeaIntakeRoller = new SparkMax(1, MotorType.kBrushless);
        algeaIntakeActuator = new SparkMax(2, MotorType.kBrushless);

        pidController = algeaIntakeActuator.getClosedLoopController();
    }

    public static AlgaeIntakeSubsystem get() {
        return instance;
    }

    public Command algaeRoller(AlgaeIntakeSubsystem algaeIntakeSubsystem, DoubleSupplier forward,
            DoubleSupplier reverse) {
        return Commands.run(
                () -> algeaIntakeRoller.set(forward.getAsDouble() - reverse.getAsDouble()), algaeIntakeSubsystem);
    }

    // TODO ajust the tic values
    public void setTargetTics(double tics) {
        targetTics = MathUtil.clamp(tics, algaeActuatorConstants.Max_Tics, algaeActuatorConstants.Min_Tics);
    }

    // TODO check the motor to see if needed to be inverted
    public Command algaeActuator(double tics) {
        return Commands.runOnce(() -> {
            if (min) {
                setTargetTics(algaeActuatorConstants.Max_Tics);
            } else {
                setTargetTics(algaeActuatorConstants.Min_Tics);
            }
            setAngle();
            min = !min;
        });
    }

    // TODO find gearRatio
    private void setAngle() {
        double targetRotation = (targetTics * gearRatio) / ticsPerRotation;
        pidController.setReference(targetRotation, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // public Command algaeActuator() {
    // return Commands.run(() ->{ System.out.println("working actuator");
    // },this);
    // }
}
