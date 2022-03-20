// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Index;

public class CommandMoveIndex extends CommandBase {
  Index index;
  int balls;
  double setpoint;
  public CommandMoveIndex(Index index, int balls) {
    this.index = index;
    this.balls = balls;
  }

  @Override
  public void initialize() {
    this.setpoint = index.getIndexPosition() + kControl.INDEX_ONE_BALL_ROTATIONS*balls;
  }

  @Override
  public void execute() {
    index.runClosedLoopPosition(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    index.setBallsIndexed(Math.max(0, index.getBallsIndexed()-balls));
  }

  @Override
  public boolean isFinished() {

    return Math.abs(index.getIndexPosition()-setpoint) < kControl.INDEX_ALLOWED_ERROR_ROTATIONS;
  }
}
