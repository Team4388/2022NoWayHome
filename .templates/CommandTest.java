// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc4388.robot.subsystems.*;
import org.junit.*;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class CommandTest {
    private CommandScheduler scheduler = null;

    @Before
    public void setup() {
        scheduler = CommandScheduler.getInstance();
    }

    // TODO: Update this to use an actual command. Won't work with inline commands for some reason

    @Test
    public void testExample() {
        // Arrange
        ArcadeDrive drive = mock(ArcadeDrive.class);
        RunCommand command = new RunCommand(() -> drive.driveWithInput(0, 0), drive);

        // Act
        scheduler.schedule(command);
        scheduler.run();

        // Assert
        verify(drive).driveWithInput(0, 0);
    }
}
