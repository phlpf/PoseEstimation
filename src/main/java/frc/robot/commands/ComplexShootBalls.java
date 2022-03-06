// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexShootBalls extends SequentialCommandGroup {
  /** Creates a new ComplexShootBalls. */
  public ComplexShootBalls(Shooter shooter, Index index, Acquisition acquisition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> acquisition.setRollerRPM(0)),
      new InstantCommand(() -> acquisition.extendArms()),
      new CommandRunShooter(shooter, 5200),
      new CommandIndexBalls(index, 2),
      new WaitCommand(5),
      new InstantCommand(() -> index.setBallsIndexed(0)),
      new InstantCommand(() -> shooter.setVelocity(0))
    );
  }
}
