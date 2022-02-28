package frc4388.commands;

import org.junit.Test;

import frc4388.robot.commands.AimToCenter;
import org.junit.Assert;

public class AimToCenterTest {

    private static final double DELTA = 1e-15;

    @Test
    public void givenAngle_whenTestIfDeadzone_thenReturnIfInDeadzone() {
        boolean output;

            //20 deg
        output = AimToCenter.isHardwareDeadzone(20.);
        Assert.assertFalse(output);

            //-10 deg
        output = AimToCenter.isHardwareDeadzone(-10.);
        Assert.assertTrue(output);
        
            //-1 deg
        output = AimToCenter.isHardwareDeadzone(-1.);
        Assert.assertTrue(output);

            //341 deg
        output = AimToCenter.isHardwareDeadzone(341.);
        Assert.assertTrue(output);

            //340 deg
        output = AimToCenter.isHardwareDeadzone(340.);
        Assert.assertFalse(output);
        
            //0 deg
        output = AimToCenter.isHardwareDeadzone(0.);
        Assert.assertFalse(output);
        
            //200 deg
        output = AimToCenter.isHardwareDeadzone(200.);
        Assert.assertFalse(output);

            //2000000 deg
        output = AimToCenter.isHardwareDeadzone(2000000.);
        Assert.assertTrue(output);

            //NaN deg
        output = AimToCenter.isHardwareDeadzone(Double.NaN);
        Assert.assertFalse(output);
    }

    @Test 
    public void givenOdometry_whenCalculateAngleToCenter_thenReturnAngleToCenter() {
        double actual;
        double expected;

            //(5,5) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(5., 5., 0.);
        expected = 180. + 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(-5,5) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(-5.0, 5., 0.);
        expected = 180. + 90. + 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(-5,-5) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(-5.0, -5., 0.);
        expected = 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(5,-5) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(5., -5., 0.);
        expected = 90. + 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(0,0) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(0., 0., 0.);
        Assert.assertNotNull(actual);

            //(5,5) Gyro = 180 deg
        actual = AimToCenter.angleToCenter(5., 5., 180.);
        expected = 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(100,100) Gyro = 90 deg
        actual = AimToCenter.angleToCenter(100., 100., 90.);
        expected = 90. + 45.;
        Assert.assertEquals(expected, actual, DELTA);

            //(null,5) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(Double.NaN, 5., 0.);
        expected = Double.NaN;
        Assert.assertEquals(expected, actual, DELTA);

            //(null,null) Gyro = 0 deg
        actual = AimToCenter.angleToCenter(Double.NaN, Double.NaN, 0.);
        expected = Double.NaN;
        Assert.assertEquals(expected, actual, DELTA);

            //(null,null) Gyro = null deg
        actual = AimToCenter.angleToCenter(Double.NaN, Double.NaN, Double.NaN);
        expected = Double.NaN;
        Assert.assertEquals(expected, actual, DELTA);

            //(5,5) Gyro = -20 deg
        actual = AimToCenter.angleToCenter(5., -5., -45.);
        expected = 180.;
        Assert.assertEquals(expected, actual, DELTA);
    
    }
}
