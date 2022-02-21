// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.ClimbConstants.ClimberPid;
import frc.robot.ClimbConstants;

/** Add your docs here. */
public class ClimbArm {
    private CANSparkMax angleMotor;
    private CANSparkMax reachMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder reachEncoder;
    private SparkMaxPIDController anglePidController;
    private SparkMaxPIDController reachPidController;
    public ClimbArm(int angleId, int reachId, ClimberPid anglePID, ClimberPid reachPID){
        angleMotor = new CANSparkMax(angleId, MotorType.kBrushless);
        reachMotor = new CANSparkMax(reachId, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        reachEncoder = reachMotor.getEncoder();
        anglePidController = angleMotor.getPIDController();
        reachPidController = angleMotor.getPIDController();
        ClimbConstants.addPidToMotor(anglePidController, anglePID);
        ClimbConstants.addPidToMotor(reachPidController, reachPID);
    }
}
