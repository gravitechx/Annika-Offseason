// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;
    // private final DigitalInput sensor;
    // private final DigitalInput sensor2;
    private SparkMaxConfig motorConfig;
    private IntakeState intakeState;

    /** Creates a new Intake. */
    public Intake() {
        intakeMotor = new SparkMax(5, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.disableFollowerMode();
        intakeState = IntakeState.IDLE;

        intakeMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // sensor = new DigitalInput(0);
        // sensor2 = new DigitalInput(1);
    }

    private void spin(double voltage) {
        intakeMotor.setVoltage(-voltage);
    }

    public void setState(IntakeState state) {
        intakeState = state;
        spin(intakeState.getVoltage());
    }

    @Override
    public void periodic() {
        // if (!sensor.get() || !sensor2.get()) {
        // // Lights.setColor(0, 255,61);
        // }
    }
}
