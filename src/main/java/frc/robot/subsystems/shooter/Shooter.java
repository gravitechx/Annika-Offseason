// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class Shooter extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMax motor2;
    private ShooterState shooterState;

    /** Creates a new Shooter. */
    public Shooter() {
        motor = new SparkMax(3, MotorType.kBrushless);
        motor2 = new SparkMax(17, MotorType.kBrushless);
        shooterState = ShooterState.IDLE;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter motor speed", motor.get());

    }

    private void spinNoFeed(double voltage, double secondVoltage) {
        motor.setVoltage(-voltage);
        motor2.setVoltage(secondVoltage);
    }

    public void setState(ShooterState state) {
        shooterState = state;
        spinNoFeed(shooterState.getVoltage(), shooterState.getVoltage2());

    }

    // public void spinWithFeed(double speed, double secondSpeed,Indexer indexer,
    // Intake intake){
    // motor.set(-speed);
    // motor2.set(secondSpeed);

    // indexer.spin((speed == 0) ? 0 :0.5, intake);

    // }
}
