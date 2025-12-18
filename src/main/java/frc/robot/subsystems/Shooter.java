// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMax motor2;

  /** Creates a new Shooter. */
  public Shooter() {
    motor = new SparkMax(0, MotorType.kBrushless);
    motor2 = new SparkMax(1, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter motor speed", motor.get());

  }

  public void spinNoFeed(double speed, double secondSpeed){
    motor.set(-speed);
    motor2.set(secondSpeed);
  }
  public void spinWithFeed(double speed, double secondSpeed,Indexer indexer, Intake intake){
    motor.set(-speed);
    motor2.set(secondSpeed);

    indexer.spin((speed == 0) ? 0 :0.5, intake);

    

  }
}
