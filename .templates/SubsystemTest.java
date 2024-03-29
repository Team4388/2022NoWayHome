// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import org.junit.*;

import edu.wpi.first.wpilibj.Spark;
import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

/**
 * Based off the LEDSubsystemTest class
 */
public class SubsystemTest {
    @Test
    public void testConstructor() {
        // Arrange
        Spark ledController = mock(Spark.class);

        // Act
        LED led = new LED(ledController);

        // Assert
        assertEquals(LEDConstants.DEFAULT_PATTERN.getValue(), led.getPattern().getValue(), 0.0001);
    }

    @Test
    public void testPatterns() {
        // Arrange
        Spark ledController = mock(Spark.class);
        LED led = new LED(ledController);

        // Act
        led.setPattern(LEDPatterns.RAINBOW_RAINBOW);

        // Assert
        assertEquals(LEDPatterns.RAINBOW_RAINBOW.getValue(), led.getPattern().getValue(), 0.0001);

        // Act
        led.setPattern(LEDPatterns.BLUE_BREATH);

        // Assert
        assertEquals(LEDPatterns.BLUE_BREATH.getValue(), led.getPattern().getValue(), 0.0001);

        // Act
        led.setPattern(LEDPatterns.SOLID_BLACK);

        // Assert
        assertEquals(LEDPatterns.SOLID_BLACK.getValue(), led.getPattern().getValue(), 0.0001);
    }
}
