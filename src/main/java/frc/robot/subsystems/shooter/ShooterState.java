package frc.robot.subsystems.shooter;

public enum ShooterState {
    IDLE(0, 0),
    FORWARDS(-8, -8),
    BACKWARDS(8, 8);

    private double voltage;
    private double voltage2;

    private ShooterState(double voltage, double voltage2) {
        this.voltage = voltage;
        this.voltage2 = voltage2;

    }

    public double getVoltage() {
        return this.voltage;
    }

    public double getVoltage2() {
        return this.voltage2;
    }

}
