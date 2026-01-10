// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class Indexer extends SubsystemBase {
    private final SparkMax indexerMotor;
    private final SparkMax indexerMotor2;
    private IndexerState indexerState;

    /** Creates a new Indexer. */
    public Indexer() {
        indexerMotor = new SparkMax(20, MotorType.kBrushless);
        indexerMotor2 = new SparkMax(4, MotorType.kBrushless);
        indexerState = IndexerState.IDLE;

    }

    // private void spin(double speed, Intake intake) {
    // intake.spin(speed);
    // indexerMotor.set(speed);
    // indexerMotor2.set(-speed);
    // }

    private void spinNoIntake(double voltage) {
        indexerMotor.setVoltage(voltage);
        indexerMotor2.setVoltage(-voltage);
    }

    public void setState(IndexerState state) {
        indexerState = state;
        spinNoIntake(indexerState.getVoltage());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
