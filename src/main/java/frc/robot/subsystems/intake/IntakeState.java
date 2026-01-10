package frc.robot.subsystems.intake;

public enum IntakeState {
    IDLE(0),
    INTAKING(8),
    OUTAKING(-8);

    private double voltage;

    private IntakeState(double voltage) {
        this.voltage = voltage;

    }

    public double getVoltage() {
        return this.voltage;
    }

}
