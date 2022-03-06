// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.XboxController.Button;

/** Add your docs here. */
public  class kClimb {
    public static class ClimberPid{
        public double p;
        public double i;
        public double d;
        public double ff;
        public double iz;
        public double min;
        public double max;
        public ClimberPid(double p, double i, double d, double ff, double iz, double min, double max){
            this.p = p;
            this.i = i;
            this.d = d;
            this.ff = ff;
            this.iz = iz;
            this.min = min;
            this.max = max;
        }
    }
    public static final ClimberPid climbAngleInner = new ClimberPid(0.05,
        0,
        10,
        0.0,
        0.0,
        -0.6,
        0.6);
    public static final ClimberPid climbReachInner = new ClimberPid(0.1,
        0,
        3,
        0.0,
        0.0,
        -0.5,
        0.5
        );
    public static final ClimberPid climbAngleOuter = new ClimberPid(0.05,
        0,
        10,
        0.0,
        0.0,
        -0.6,
        0.6);
    public static final ClimberPid climbReachOuter = new ClimberPid(0.1,
        0,
        3,
        0.0,
        0.0,
        -0.5,
        0.5);// TODO: Add real values
    public static void addPidToMotor(SparkMaxPIDController controller, ClimberPid pid){
        controller.setP(pid.p);
        controller.setI(pid.i);
        controller.setD(pid.d);
        controller.setFF(pid.ff);
        controller.setIZone(pid.iz);
        controller.setOutputRange(pid.min, pid.max);
    }

    public static final int CLIMB_BUTTON = Button.kA.value;
    public static final double CLIMB_ROTATION_TO_INCH = 1/5.555;
    public static final double CLIMB_ROTATION_TO_DEGREE = 1/1.111111111111111111111;
    public static final double CLIMB_MAX_EXTEND = 24;
    public static final double CLIMB_MIN_EXTEND = 0;
    public static final double CLIMB_REACH_ALLOWED_ERROR = 3; 
    public static final double CLIMB_ANGLE_ALLOWED_ERROR_EXACT = 0.25; 
    public static final double CLIMB_ANGLE_ALLOWED_ERROR_GENERAL = 2; 

    public static final double INNER_NOLOAD_STALL_CURRENT_REACH = 18;
    public static final double INNER_NOLOAD_STALL_CURRENT_ANGLE = 9;
    public static final int ANGLE_SMART_CURRENT = 5;
}

