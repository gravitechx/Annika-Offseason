// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final DigitalInput sensor;
  private final DigitalInput sensor2;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor= new SparkMax(0, MotorType.kBrushless);
    sensor= new DigitalInput(0);
    sensor2= new DigitalInput(1);
  }

  public void spin(double speed){
    intakeMotor.set(-speed);
  }

  @Override
  public void periodic() {
    if(!sensor.get() || !sensor2.get()){
      // Lights.setColor(0, 255,61);
    }
  }
}
