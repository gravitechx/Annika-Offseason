// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final SparkMax indexerMotor;
    private final SparkMax indexerMotor2;

  /** Creates a new Indexer. */
  public Indexer() {
        indexerMotor= new SparkMax(0, MotorType.kBrushless);
        indexerMotor2= new SparkMax(1, MotorType.kBrushless);

  }

  public void spin(double speed){
    indexerMotor.set(-speed);
    indexerMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
