// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRPMS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexSpinUpShooter extends SequentialCommandGroup {
  /** Creates a new ComplexSpinUpShooter. */
  public ComplexSpinUpShooter(Shooter shooter, Acquisition acquisition, ShooterRPMS rpms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> acquisition.extendArms()),
      new CommandRunShooter(shooter, rpms, true)
    );
  }
}
