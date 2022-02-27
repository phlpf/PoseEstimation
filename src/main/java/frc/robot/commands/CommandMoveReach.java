// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.utils.ClimberArm;

public class CommandMoveReach extends CommandBase {
  /** Creates a new CommandSetReach. */
  private ClimberArm arm;
  private double position;
  private boolean useCurrentLimits;
  public CommandMoveReach(ClimberArm arm, double position, boolean useCurrentLimits){
    this.arm = arm;
    this.position = position;
    this.useCurrentLimits = useCurrentLimits;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setReachSetpoint(position/kClimb.CLIMB_ROTATION_TO_INCH);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveReach(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double reachError = arm.calculateReachError();
    SmartDashboard.putNumber("Reach Error", reachError);
    // Check for current spike
    boolean isAtStop = (arm.getReachCurrent() > kClimb.INNER_NOLOAD_STALL_CURRENT && useCurrentLimits);
    return reachError < kClimb.CLIMB_REACH_ALLOWED_ERROR || isAtStop;
  }
}
