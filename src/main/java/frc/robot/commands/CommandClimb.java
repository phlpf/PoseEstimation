// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ClimbConstants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandClimb extends SequentialCommandGroup {
  /** Creates a new CommandClimb. */
  public CommandClimb(Climber climber, XboxController gamepad) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> climber.extendArm(climber.outerArm, ClimbConstants.CLIMB_MAX_EXTEND)),
      new CommandWaitForButton(gamepad, ClimbConstants.CLIMB_BUTTON),
      new InstantCommand(()-> climber.extendArm(climber.outerArm, 0))
    );
  }
}   
