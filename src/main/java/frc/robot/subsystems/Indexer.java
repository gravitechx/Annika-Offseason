// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerState;

public class Indexer extends SubsystemBase {
    private final SparkMax indexerMotor;
    private final SparkMax indexerMotor2;
    private IndexerState indexerState;

  /** Creates a new Indexer. */
  public Indexer() {
        indexerMotor= new SparkMax(20, MotorType.kBrushless);
        indexerMotor2= new SparkMax(4, MotorType.kBrushless);
        indexerState = IndexerState.IDLE;
  }

  public void spin(double speed) {
    indexerMotor.setVoltage(speed);
    indexerMotor2.setVoltage(-speed);
  }

  public void setState(IndexerState state){
    indexerState = state;
    spin(indexerState.getVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
