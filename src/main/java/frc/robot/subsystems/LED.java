// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final DigitalOutput bitOne = new DigitalOutput(13);
  private final DigitalOutput bitTwo = new DigitalOutput(14);
  private final DigitalOutput bitThree = new DigitalOutput(15);
  private final DigitalOutput bitFour = new DigitalOutput(16);
  public LED() {
    
  }

  @Override
  public void periodic() {
    //bitOne.set(DriverStation.getAlliance() == DriverStation.Alliance.Red);

    // pull enabled port high if enabled, low if disabled
    //bitTwo.set(DriverStation.isEnabled());
  
    // pull auto port high if in autonomous, low if in teleop (or disabled)
    //bitThree.set(DriverStation.isAutonomous());
  
    // pull alert port high if match time remaining is between 30 and 25 seconds
    //var matchTime = DriverStation.getMatchTime();
    //bitFour.set(matchTime <= 30 && matchTime >= 25);  // This method will be called once per scheduler run
  }

//ALLIANCERED = 0b0000
//ALLIANCEBLUE = 0b0001

  public void arduinoPattern(int num){
    bitOne.set((num & 0b0001) == 0b0001);
    bitTwo.set((num & 0b0010) == 0b0010);
    bitThree.set((num & 0b0100) == 0b0100);
    bitFour.set((num & 0b1000) == 0b1000);
  }
}
  