// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ExtenderIntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtenderIntakeGroup extends ParallelRaceGroup {
  
  public static int direction;

  /** Creates a new RunExtenderAndIntake. */
  public ExtenderIntakeGroup(Intake intake, Extender extender) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    ExtenderIntakeGroup.direction = 1; // assume extender starts retracted completely
    addCommands(new RunIntakeConditionally(intake), new RunExtender(extender));
  }

  public static void changeDirection() { // Never implemented?
    ExtenderIntakeGroup.direction *= -1;
  }
}
