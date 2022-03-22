// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;


public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motorFront;
  private RelativeEncoder encoderFront;
  private SparkMaxPIDController pidFront;
  private CANSparkMax motorBack;
  private RelativeEncoder encoderBack;
  private SparkMaxPIDController pidBack;

  private double setpointVelocityFront = 0;
  private double setpointVelocityBack = 0;

  public Shooter() {
    motorFront = new CANSparkMax(kCANIDs.SHOOTER_MOTOR_FRONT, MotorType.kBrushless);
    motorFront.restoreFactoryDefaults();
    motorFront.setInverted(true);
    motorFront.setIdleMode(IdleMode.kCoast);

    encoderFront = motorFront.getEncoder();

    pidFront = motorFront.getPIDController();
    pidFront.setP(2.5e-5);
    pidFront.setI(8.5e-7);
    pidFront.setD(0.00001);
    pidFront.setFF(0.000015);
    pidFront.setIZone(0);
    pidFront.setOutputRange(-1,1);

    motorBack = new CANSparkMax(kCANIDs.SHOOTER_MOTOR_BACK, MotorType.kBrushless);
    motorBack.restoreFactoryDefaults();
    motorBack.setInverted(false);
    motorBack.setIdleMode(IdleMode.kCoast);

    encoderBack = motorBack.getEncoder();

    pidBack = motorBack.getPIDController();
    pidBack.setP(9.5e-5);
    pidBack.setI(8e-7);
    pidBack.setD(0);
    pidBack.setFF(0.000015);
    pidBack.setIZone(0);
    pidBack.setOutputRange(-1,1);
  }

  public void setVelocityFront(double setpoint) {
    setpointVelocityFront = setpoint;
    if(setpointVelocityFront != 0){
      pidFront.setReference(setpointVelocityFront, CANSparkMax.ControlType.kVelocity);
    }
    else{
      motorFront.set(0);
    }
  }
  public void setVelocityBack(double setpoint) {
    setpointVelocityBack = setpoint;
    if(setpointVelocityBack != 0){
      pidBack.setReference(setpointVelocityBack, CANSparkMax.ControlType.kVelocity);
    }
    else{
      motorBack.set(0);
    }
  }
  public double getVelocityFront() {
    return encoderFront.getVelocity();
  }
  public double getVelocityBack() {
    return encoderBack.getVelocity();
  }

  public void setPercentOut(double percent) {
    motorFront.set(percent);
    motorBack.set(percent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("A-ShtFrnt", motorFront.getOutputCurrent());
    SmartDashboard.putNumber("shooter/actual Velocity Shooter Front", encoderFront.getVelocity());
    SmartDashboard.putNumber("shooter/setpoint Velocity Shooter", setpointVelocityFront);
    SmartDashboard.putNumber("A-ShtBck", motorBack.getOutputCurrent());
    SmartDashboard.putNumber("shooter/actual Velocity Shooter Back", encoderBack.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public static class ShooterRPMS{
    public double RPMFront;
    public double RPMBack;
    public ShooterRPMS(double RPMFront, double RPMBack){
      this.RPMBack = RPMBack;
      this.RPMFront = RPMFront;
    }
  }
}