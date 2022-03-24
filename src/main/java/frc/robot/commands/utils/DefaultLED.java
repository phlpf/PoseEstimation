// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Acquisition;

public class DefaultLED extends CommandBase {
  private final LED led;
  private final Index index;
  private final Acquisition acquisition;
  private int pattern;
  /** Creates a new DefaultLED. */
  public DefaultLED(LED led, Index index, Acquisition acquisition) {
    this.led = led;
    this.index = index;
    this.acquisition = acquisition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isAutonomous()){
      pattern = 1;
    }
    if(DriverStation.isTeleopEnabled()){
      pattern = 2;
    }
    // switch (index.getBallsIndexed()) {
    //   case value:
        
    //     break;
    
    //   default:
    //     break;
    // }
    if(index.getBallsIndexed() == 0){
      pattern = 3;
      if(acquisition.areArmsExtended()){
        pattern = 6;
      }
    }
    if(index.getBallsIndexed() == 1){
      pattern = 4;
      if(acquisition.areArmsExtended()){
        pattern = 7;
      }
    }
    if(index.getBallsIndexed() == 2){
      pattern = 5;
      if(acquisition.areArmsExtended()){
        pattern = 8;
      }
    }
    var matchTime = DriverStation.getMatchTime();
    if(matchTime <= 45 && matchTime >= 40){
      pattern = 9;
    }
    //if(fistbar){
    //   pattern = 10;
    // }    
    //if(secondbar){
    //   pattern = 11;
    // }    
    //if(thirdbar){
    //   pattern = 12;
    // }    
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
