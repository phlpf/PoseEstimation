// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;


public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;

  private double setpointVelocity = 0;

  public Shooter() {
    motor = new CANSparkMax(kCANIDs.SHOOTER_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kCoast);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 1000);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 60);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000);
    motor.setControlFramePeriodMs(40);

    encoder = motor.getEncoder();

    pid = motor.getPIDController();
    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setFF(0.000156);
    pid.setIZone(0);
    pid.setOutputRange(-1,1);
  }

  public void setVelocity(double setpoint) {
    setpointVelocity = setpoint;
  }
  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setPercentOut(double percent) {
    motor.set(percent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("A-Sht", motor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/actual Velocity Shooter", encoder.getVelocity());
    SmartDashboard.putNumber("shooter/setpoint Velocity Shooter", setpointVelocity);
    pid.setReference(setpointVelocity, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}