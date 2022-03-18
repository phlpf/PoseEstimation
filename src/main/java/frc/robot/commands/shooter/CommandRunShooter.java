// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class CommandRunShooter extends CommandBase {
  /** Creates a new CommandRunShooter. */
  Shooter shooter;
  double RPMFront;
  double RPMBack;
  public CommandRunShooter(Shooter shooter, double RPMFront, double RPMBack) {
    this.shooter = shooter;
    this.RPMFront = RPMFront;
    this.RPMBack = RPMBack;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocityFront(RPMFront);
    shooter.setVelocityBack(RPMBack);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooter.getVelocityFront() - RPMFront) < 100 && Math.abs(shooter.getVelocityBack() - RPMBack) < 100;
  }
}
