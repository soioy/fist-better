// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX motor1;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //motor1 = new TalonFX(7, motor1.kBrushless);
  }

  public void runMotor(double speed) {
    motor1.set(speed);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}