// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Seek extends SequentialCommandGroup {

  /** Seeks. 
   * Seeking -> Sought
   * @author Aarav Shah
   * @blame Aarav Shah (thomas did this)
  */
  public Seek(SwerveDrive swerve, BoomBoom drum, Turret turret, Hood hood, VisionOdometry visionOdometry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Shoot(swerve, drum, turret, hood, false), new TrackTarget(turret, drum, hood, visionOdometry));
  }
}
