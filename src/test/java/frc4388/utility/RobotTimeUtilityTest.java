// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import static org.junit.Assert.*;

import org.junit.*;

/**
 * Add your docs here.
 */
public class RobotTimeUtilityTest {
  RobotTime robotTime = RobotTime.getInstance();

  @Test
  public void testUpdateTimes() {
    // Arrange
    long lastTime;
    robotTime.m_deltaTime = 0;
    robotTime.m_robotTime = 0;
    robotTime.m_lastRobotTime = 0;
    robotTime.m_frameNumber = 0;
    robotTime.endMatchTime();
    robotTime.m_lastMatchTime = 0;

    // Assert
    assertEquals(0, robotTime.m_deltaTime);
    assertEquals(0, robotTime.m_robotTime);
    assertEquals(0, robotTime.m_lastRobotTime);
    assertEquals(0, robotTime.m_frameNumber);
    lastTime = robotTime.m_robotTime;

    // Act
    wait(1);
    robotTime.updateTimes();

    // Assert
    assertEquals(true, robotTime.m_deltaTime > 0);
    assertEquals(true, robotTime.m_robotTime > 0);
    assertEquals(lastTime, robotTime.m_lastRobotTime);
    assertEquals(1, robotTime.m_frameNumber);
    lastTime = robotTime.m_robotTime;

    // Act
    wait(1);
    robotTime.updateTimes();

    // Assert
    assertEquals(true, robotTime.m_deltaTime > 0);
    assertEquals(true, robotTime.m_robotTime > 0);
    assertEquals(lastTime, robotTime.m_lastRobotTime);
    assertEquals(2, robotTime.m_frameNumber);
  }

  @Test
  public void testMatchTime() {
    // Arrange
    long lastTime;

    // Assert
    assertEquals(0, robotTime.m_matchTime);
    assertEquals(0, robotTime.m_lastMatchTime);
    lastTime = robotTime.m_matchTime;

    // Act
    robotTime.startMatchTime();
    wait(1);
    robotTime.updateTimes();

    // Assert
    assertEquals(true, robotTime.m_matchTime > 0);
    assertEquals(lastTime, robotTime.m_lastMatchTime);
    lastTime = robotTime.m_matchTime;
    
    // Act
    wait(1);
    robotTime.updateTimes();
    robotTime.endMatchTime();

    // Assert
    assertEquals(0, robotTime.m_matchTime);
    assertEquals(lastTime, robotTime.m_lastMatchTime);
    lastTime = robotTime.m_matchTime;

    // Act
    wait(1);
    robotTime.updateTimes();

    // Assert
    assertEquals(0, robotTime.m_matchTime);
    assertEquals(lastTime, robotTime.m_lastMatchTime);
  }

  private void wait(int millis) {
    try {
      Thread.sleep(millis);
    } catch (Exception e) {}
  }
}
