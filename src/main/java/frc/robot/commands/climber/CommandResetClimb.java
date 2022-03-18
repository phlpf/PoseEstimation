// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.CommandMoveAngle.CurrentLimit;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandResetClimb extends SequentialCommandGroup {
  /** Creates a new CommandResetClimb. */
  public CommandResetClimb(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new CommandMoveReach(climber.outerArm, 0, true),
        new CommandMoveReach(climber.innerArm, 0, true)
      ),
      new ParallelCommandGroup(
        new CommandMoveAngle(climber.outerArm, 0, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT),
        new CommandMoveAngle(climber.innerArm, 0, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT)
      )
    );
  }
}
