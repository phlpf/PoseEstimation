// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ClimbConstants {
    public class ClimberPid{
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
    public ClimberPid Angle1 = new ClimberPid(0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
    public ClimberPid Reach1 = new ClimberPid(0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
    public ClimberPid Angle2 = new ClimberPid(0.0,
       0.0,
       0.0,
       0.0,
       0.0,
       0.0,
       0.0); // TODO: Add real values
}
