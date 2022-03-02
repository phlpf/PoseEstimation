// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.utils.ClimberArm;

public class CommandMoveAngle extends CommandBase {
  /** Creates a new CommandSetReach. */
  private ClimberArm arm;
  private double angle;
  private double angleErrorMin;
  private boolean useCurrentLimits;
  public CommandMoveAngle(ClimberArm arm, double angle, boolean useCurrentLimits, double angleErrorMin){
    this.arm = arm;
    this.angle = angle;
    this.useCurrentLimits = useCurrentLimits;
    this.angleErrorMin = angleErrorMin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setAngleSetpoint(angle/kClimb.CLIMB_ROTATION_TO_DEGREE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveAnglePOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleError = arm.calculateAngleError();
    SmartDashboard.putNumber("angle Error", angleError);
    // Check for current spike
    boolean isAtStop = (useCurrentLimits && arm.getAngleCurrent() > kClimb.INNER_NOLOAD_STALL_CURRENT_ANGLE);
    if(isAtStop){System.out.println("Current limit reached, at stop: " + arm.getAngleCurrent());}
    System.out.println("Current: " + arm.getAngleCurrent());
    return angleError < angleErrorMin || isAtStop;
  }
}
