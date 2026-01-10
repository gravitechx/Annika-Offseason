package frc.robot.subsystems.indexer;

public enum IndexerState {
    IDLE(0),
    FORWARDS(5),
    BACKWARDS(-5);

    private double voltage;

    private IndexerState(double voltage) {
        this.voltage = voltage;

    }

    public double getVoltage() {
        return this.voltage;
    }

}
