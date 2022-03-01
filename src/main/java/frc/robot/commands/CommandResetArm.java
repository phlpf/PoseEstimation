// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.utils.ClimberArm;

public class CommandResetArm extends CommandBase {
  /** Creates a new ResetArm. */
  private ClimberArm arm;
  private ArrayList<Double> currents = new ArrayList<Double>();
  public CommandResetArm(ClimberArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveAngle(-0.35);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getAngleCurrent() > kClimb.INNER_NOLOAD_STALL_CURRENT_REACH;
  }
}
