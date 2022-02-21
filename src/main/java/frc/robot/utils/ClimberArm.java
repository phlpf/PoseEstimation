// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.ClimbConstants.ClimberPid;
import frc.robot.ClimbConstants;

/** Add your docs here. */
public class ClimberArm {
    // private CANSparkMax angleMotor;
    private CANSparkMax reachMotor;
    // private RelativeEncoder angleEncoder;
    public RelativeEncoder reachEncoder;
    // private SparkMaxPIDController anglePidController;
    private SparkMaxPIDController reachPidController;
    public ClimberArm(int angleId, int reachId, ClimberPid anglePID, ClimberPid reachPID){
        // angleMotor = new CANSparkMax(angleId, MotorType.kBrushless);
        reachMotor = new CANSparkMax(reachId, MotorType.kBrushless);
        reachMotor.restoreFactoryDefaults();
        reachMotor.setInverted(false);
        // angleEncoder = angleMotor.getEncoder();
        reachEncoder = reachMotor.getEncoder();
        reachEncoder.setPosition(0);
        // anglePidController = angleMotor.getPIDController();
        reachPidController = reachMotor.getPIDController();
        // ClimbConstants.addPidToMotor(anglePidController, anglePID);
        ClimbConstants.addPidToMotor(reachPidController, reachPID);
    }

    public void setPositionSetpoint(double rotations){
        reachPidController.setReference(rotations, ControlType.kPosition);
    }
}
