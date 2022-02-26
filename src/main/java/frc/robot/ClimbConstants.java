// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.Climber;

/** Add your docs here. */
public  class ClimbConstants {
    public static class ClimberPid{
        double p;
        double i;
        double d;
        double ff;
        double iz;
        double min;
        double max;
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
    public static ClimberPid climbAngleInner = new ClimberPid(0.75,
        2e-5,
        3,
        0.0,
        0.0,
        -0.3,
        0.3);
    public static ClimberPid climbReachInner = new ClimberPid(0.75,
        2e-5,
        3,
        0.0,
        0.0,
        -0.3,
        0.3);
    public static ClimberPid climbAngleOuter = new ClimberPid(0.75,
        2e-5,
        3,
        0.0,
        0.0,
        -0.3,
        0.3);
    public static ClimberPid climbReachOuter = new ClimberPid(0.75,
        2e-5,
        3,
        0.0,
        0.0,
        -0.3,
        0.3); // TODO: Add real values
    public static void addPidToMotor(SparkMaxPIDController controller, ClimberPid pid){
        controller.setP(pid.p);
        controller.setI(pid.i);
        controller.setD(pid.d);
        controller.setFF(pid.ff);
        controller.setIZone(pid.iz);
        controller.setOutputRange(pid.min, pid.max);
    }

    public static final int INNER_ANGLE_ID = 15;
    public static final int INNER_REACH_ID = 14;
    public static final int OUTER_ANGLE_ID = 17;
    public static final int OUTER_REACH_ID = 16;
    public static final int CLIMB_BUTTON = Button.kA.value;
    public static final double CLIMB_ROTATION_TO_INCH = 1/5.555;
    public static final double CLIMB_ROTATION_TO_DEGREE = 3.25;
    public static final double CLIMB_MAX_EXTEND = 24;
    public static final double CLIMB_MIN_EXTEND = 0.5;
    public static final double CLIMB_REACH_ALLOWED_ERROR = 1; 
}
