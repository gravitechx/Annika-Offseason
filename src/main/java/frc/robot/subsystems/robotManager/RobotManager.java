package frc.robot.subsystems.robotManager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotManager extends SubsystemBase {
    
    public RobotState robotState = RobotState.IDLE;
    public Intake intakeSub = new Intake();
    public Indexer indexerSub = new Indexer();
    public Shooter shooterSub = new Shooter();

    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
        new SequentialCommandGroup(
                new InstantCommand(() -> intakeSub.setState(robotState.getIntakeState())),
                new InstantCommand(() -> indexerSub.setState(robotState.getIndexerState())),
                new InstantCommand(() -> shooterSub.setState(robotState.getShooterState()))).schedule();

    }

    public void periodic() {
        DogLog.log("Robot State", robotState);

    }
}