// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.ClimbConstants.ClimberPid;
import frc.robot.ClimbConstants;

/** Add your docs here. */
public class ClimberArm {
    private CANSparkMax angleMotor;
    private CANSparkMax reachMotor;
    public RelativeEncoder angleEncoder;
    public RelativeEncoder reachEncoder;
    private SparkMaxPIDController anglePidController;
    private SparkMaxPIDController reachPidController;
    private double reachSetpoint = 0;
    private double angleSetpoint = 0;
    public ClimberArm(int angleId, int reachId, ClimberPid anglePID, ClimberPid reachPID, boolean isReversedReach){
        angleMotor = new CANSparkMax(angleId, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(IdleMode.kBrake);

        reachMotor = new CANSparkMax(reachId, MotorType.kBrushless);
        reachMotor.restoreFactoryDefaults();
        reachMotor.setInverted(isReversedReach);
        reachMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPosition(0);
        
        reachEncoder = reachMotor.getEncoder();
        reachEncoder.setPosition(0);
        
        anglePidController = angleMotor.getPIDController();
        reachPidController = reachMotor.getPIDController();
        ClimbConstants.addPidToMotor(anglePidController, anglePID);
        ClimbConstants.addPidToMotor(reachPidController, reachPID);

        // Set limits for reach
        reachMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(ClimbConstants.CLIMB_MIN_EXTEND/ClimbConstants.CLIMB_ROTATION_TO_INCH));
        reachMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        reachMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(ClimbConstants.CLIMB_MAX_EXTEND/ClimbConstants.CLIMB_ROTATION_TO_INCH));
        reachMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        
        // Set soft limits for angle
        angleMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(-26/ClimbConstants.CLIMB_ROTATION_TO_DEGREE));
        angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        angleMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(45/ClimbConstants.CLIMB_ROTATION_TO_DEGREE));
        angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        setReachSetpoint(ClimbConstants.CLIMB_MIN_EXTEND/ClimbConstants.CLIMB_ROTATION_TO_INCH);

    }

    public void setReachSetpoint(double rotations){
        reachPidController.setReference(rotations, ControlType.kPosition);
        reachSetpoint = rotations;
    }
    public void setAngleSetpoint(double rotations){
        anglePidController.setReference(rotations, ControlType.kPosition);
        angleSetpoint = rotations;
    }

    public void periodic(){
        // Stop when we are at a acceptable error
        double reachError = Math.abs(reachSetpoint - reachEncoder.getPosition());
        if(reachError < ClimbConstants.CLIMB_REACH_ALLOWED_ERROR){
            reachSetpoint = reachEncoder.getPosition();
            reachPidController.setReference(reachSetpoint, ControlType.kPosition);
        }
        double angleError = Math.abs(angleSetpoint - angleEncoder.getPosition());
        if(angleError < ClimbConstants.CLIMB_REACH_ALLOWED_ERROR){
            angleSetpoint = reachEncoder.getPosition();
            anglePidController.setReference(reachSetpoint, ControlType.kPosition);
        }
    }
    public void setAngleToCoast(){
        angleMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setReachToCoast(){
        reachMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setAngleToBrake(){
        angleMotor.setIdleMode(IdleMode.kBrake);
    }
}
