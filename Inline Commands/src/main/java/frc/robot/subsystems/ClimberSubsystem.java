package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
    
public class ClimberSubsystem extends SubsystemBase{
    private final SparkMax climber;
    
    public ClimberSubsystem() {
        climber = new SparkMax(0, null);
        

    }
}
