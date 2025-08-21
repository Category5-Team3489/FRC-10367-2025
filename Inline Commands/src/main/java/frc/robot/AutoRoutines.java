package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANDriveSubsystem;


public class AutoRoutines {

    private final CANDriveSubsystem drive;

    public AutoRoutines(AutoFactory factory, CANDriveSubsystem drive) {
        this.factory = factory;
        this.drive = drive;

    }

    private Command score(double scoreVolts) {
        return Commands.deadline(
                Commands.waitSeconds(1),
                drive.tankDrive(drive,() -> 0.0, () -> 0.0));
    }

    private Command wait(Double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                drive.tankDrive(drive,() -> 0.0, () -> 0.0));
    }

    
    public Command reverse() {
        String trajectory = "Reverse";

        return Commands.sequence(
                factory.resetOdometry(trajectory),
                factory.trajectoryCmd(trajectory));
    }

    public Command autoTest() {
        String trajectory = "AutoTest";

        return Commands.sequence(
                factory.resetOdometry(trajectory),
                factory.trajectoryCmd(trajectory));
    }

    public Command Shpeal() {
        String trajectory = "SMtoCGH";

        return Commands.sequence(
                factory.resetOdometry(trajectory),
                factory.trajectoryCmd(trajectory),
                wait(0.15),
                score(6));
    }

    public Command Wailmer() {
        String trajectory1 = "SLtoCIJ";
        String trajectory2 = "CIJtoHL";
        String trajectory3 = "HLtoCKL";

        return Commands.sequence(
                factory.resetOdometry(trajectory1),
                wait(0.01),
                factory.resetOdometry(trajectory1),
                wait(0.01),
                factory.trajectoryCmd(trajectory1),
                wait(0.15),
                score(5),
                wait(0.15),
                factory.trajectoryCmd(trajectory2),
                wait(.15),
                factory.trajectoryCmd(trajectory3),
                wait(0.15),
                score(6));
    }

    public Command Seel() {
        String trajectory1 = "SRtoCEF";
        String trajectory2 = "CEFtoHR";
        String trajectory3 = "HRtoCCD";

        return Commands.sequence(
                factory.resetOdometry(trajectory1),
                wait(0.1),
                factory.resetOdometry(trajectory1),
                wait(0.1),
                factory.trajectoryCmd(trajectory1),
                wait(0.15),
                score(5),
                wait(0.15),
                factory.trajectoryCmd(trajectory2),
                wait(.5),
                factory.trajectoryCmd(trajectory3),
                wait(0.15),
                score(6));
        // time elapsed AS OF 2/27 - 14.8 sec
    }
}
