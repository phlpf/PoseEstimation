// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kDIO;


public class Index extends SubsystemBase {
  public CANSparkMax motor;
  public RelativeEncoder encoder;
  public SparkMaxPIDController pidController;
  private DigitalInput beambreak;

  public Index() {
    motor = new CANSparkMax(kCANIDs.IDX_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    encoder = motor.getEncoder();
    pidController = motor.getPIDController();
    pidController.setP(0.1);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setOutputRange(-0.5, 0.5);

    beambreak = new DigitalInput(kDIO.BEAMBREAK);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("A-Idx", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Beam-Idx", beambreak.get());
  }

  public void setReference(double rotations){
    
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}