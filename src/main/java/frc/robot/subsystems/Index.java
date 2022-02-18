// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class Index extends SubsystemBase {
  public CANSparkMax motor;
  public RelativeEncoder encoder;
  public SparkMaxPIDController pidController;
  /** Creates a new ExampleSubsystem. */
  public Index() {
    motor = new CANSparkMax(11, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidController = motor.getPIDController();
    pidController.setP(0.1);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setReference(double rotations){
    
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}