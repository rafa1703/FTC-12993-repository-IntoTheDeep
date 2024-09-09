package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

@Config // Allows dashboard tune
public class Globals
{
    public static LynxModule chub, expHub;
    public static double EPSILON_DELTA = 0.005;
    // Global constants
    public static double
            GAMEPAD_TRIGGER_THRESHOLD = 0.2;

    public static double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        else if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public static double normalizeRadians(double radians)
    {
        radians %= 2.0 * Math.PI;
        if (radians < -Math.PI) radians += 2.0 * Math.PI;
        if (radians > Math.PI) radians -= 2.0 * Math.PI;
        return radians;
    }

    public static double normalizeDegrees(double degrees) {
        if (degrees > 180) {
            degrees -= 360;
        }
        else if (degrees < -180) {
            degrees += 360;
        }
        // keep in mind that the result is in degrees
        return degrees;
    }

    /**
     * @param current          current power returned from the pid
     * @param prev             previous power set to the motor
     * @param cachingTolerance threshold for the abs difference between the powers
     * @param motor            the motor the power should bet set if one of the conditions is true
     * @return the value to be cached as the previous power
     */
    public static double motorCaching(double current, double prev, double cachingTolerance, DcMotor motor)
    {
        // This can also be done as wrapper to the hardware object
        if (
                (Math.abs(current - prev) > cachingTolerance) ||
                        (current == 0.0 && prev != 0.0) ||
                        (current >= 1.0 && !(prev >= 1.0)) ||
                        (current <= -1.0 && !(prev <= -1.0))
        )
        {

            prev = current;
            motor.setPower(current);
        }
        return prev;
    }

    public static int TargetCaching(int target, int prevTarget, double cachingTolerance, DcMotor motor, boolean firstCycle)
    {
        if (
                (Math.abs(target - prevTarget) > cachingTolerance) || firstCycle
        )
        {
            motor.setTargetPosition(target);
            prevTarget = target;
        }

        return prevTarget;
    }

    public static double servoCaching(double current, double prev, double cachingTolerance, ServoImplEx servo)
    {
        if (
                (Math.abs(current - prev) > cachingTolerance) ||
                        (current == 0.0 && prev != 0.0) ||
                        (current >= 1.0 && !(prev >= 1.0))
        )
        {

            prev = current;
            servo.setPosition(current);
        }

        return prev;
    }
    public static DcMotor.RunMode runModeCaching(DcMotor.RunMode runMode, DcMotor.RunMode prevRunMode, DcMotor motor)
    {
        if (runMode != prevRunMode)
        {
            motor.setMode(runMode);
            prevRunMode = runMode;
        }
        return prevRunMode;
    }

    /**
     *
     * @param hubs the list of all hubs
     * @return returns a new list where chub is always at index 0
     */
    public static List<LynxModule> setHubs(List<LynxModule> hubs)
    {
        if (hubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(hubs.get(0).getSerialNumber()))
        {
            chub = hubs.get(0);
            expHub = hubs.get(1);
        } else
        {
            chub = hubs.get(1);
            expHub = hubs.get(0);
        }
        hubs.clear();
        hubs.add(chub);
        hubs.add(expHub);
        return hubs;
    }

}


