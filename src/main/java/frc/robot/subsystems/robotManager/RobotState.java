package frc.robot.subsystems.robotManager;

import frc.robot.subsystems.indexer.IndexerState;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.ShooterState;

public enum RobotState {
    IDLE(IntakeState.IDLE, IndexerState.IDLE, ShooterState.IDLE),
    INTAKING(IntakeState.INTAKING, IndexerState.IDLE, ShooterState.IDLE),
    INDEXING(IntakeState.IDLE, IndexerState.FORWARDS, ShooterState.IDLE),
    SPIN_UP_SHOOTER(IntakeState.IDLE, IndexerState.IDLE, ShooterState.FORWARDS),
    SHOOTING(IntakeState.INTAKING, IndexerState.FORWARDS, ShooterState.FORWARDS);

    private IntakeState intakeState;
    private IndexerState indexerState;
    private ShooterState shooterState;

    private RobotState(IntakeState intakeState, IndexerState indexerState, ShooterState shooterState) {
        this.intakeState = intakeState;
        this.indexerState = indexerState;
        this.shooterState = shooterState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public IndexerState getIndexerState() {
        return indexerState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

}