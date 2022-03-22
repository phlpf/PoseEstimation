// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.LED;

public class DefaultLED extends CommandBase {
  private final LED led;
  private int pattern;
  /** Creates a new DefaultLED. */
  public DefaultLED(LED led) {
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isEnabled()){
      pattern = 1;
    }
    else{
      pattern = 0;
    }
    
    led.arduinoPattern(pattern);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
