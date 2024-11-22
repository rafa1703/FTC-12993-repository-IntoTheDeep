package org.firstinspires.ftc.teamcode.system.accessory;

public class Models
{
    private static final double armFFCoeff = 0.1;
    private static final double liftMaxExtension = 600;

    public static double liftFeedForward(double liftPos)
    {
        double a = (liftPos / liftMaxExtension) * Math.PI;

        return Math.pow(Math.cos(a), 8);
    }
}
