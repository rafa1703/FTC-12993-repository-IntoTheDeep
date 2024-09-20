package org.firstinspires.ftc.teamcode.system.accessory.math;

public class Angles
{
    public static double normalizeRadians(double radians)
    {
        radians %= 2.0 * Math.PI;
        if (radians < -Math.PI) radians += 2.0 * Math.PI;
        if (radians > Math.PI) radians -= 2.0 * Math.PI;
        return radians;
    }
    public static double normalizeDegrees(double degrees)
    {
        degrees %= 360;
        if (degrees < -180) degrees += 180;
        if(degrees > 180) degrees -= 180;
        return degrees;
    }
}
