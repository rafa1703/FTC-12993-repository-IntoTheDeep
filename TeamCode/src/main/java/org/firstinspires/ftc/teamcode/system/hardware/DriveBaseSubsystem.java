package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config // Allows dashboard to tune
public class DriveBaseSubsystem
{  // no constructor for this class

    public DcMotorEx
            FL,
            FR,
            BL,
            BR;

    // higher values of k means more adjustment.
    // c centers the adjustment

//    k = 20, c = 0.7
//    x = 0.1, y = 0.000017
//    x = 0.3, y = 0.000911
//    x = 0.5, y = 0.047426
//    x = 0.7, y = 0.500000
//    x = 0.9, y = 0.977023

//    k = 5, c = 0.3
//    x = 0.1, y = 0.377541
//    x = 0.3, y = 0.500000
//    x = 0.5, y = 0.622459
//    x = 0.7, y = 0.750000
//    x = 0.9, y = 0.845534

    // where the scaling centers around
    public static double c = 1.7;
    // how agressive the scaling is
    public static double m = 0.9;
    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase = 1;
    double PowerBaseTurn = 0.88;
    double PowerStrafe = 1.1;

    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;

    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    Telemetry telemetry;
    public DriveBaseSubsystem(GeneralHardware hardware)
    {
        FL = hardware.mch0;
        FR = hardware.mch1;
        BL = hardware.mch0;
        BR = hardware.mch1;
        drivebaseSetup(true); // this has to be true for GVF, as we do glinding vectors
    }

    // this could be run in robothardware
    public void drivebaseSetup(boolean Float){
        // zero brake behavior means when motors aren't powered, they will auto brake
        if (Float)
        {
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else
        {
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        //FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setUpFloat()
    {
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static double adjustedJoystick(double x) {
        double y = Math.pow(c-x,m);
        return Math.pow(x,y);
    }

    public void Drive(double LY, double LX, double RX) {
//        double f = LY < 0? -adjustedJoystick(Math.abs(LY)):adjustedJoystick(Math.abs(LY));
//        double s = LX < 0? -adjustedJoystick(Math.abs(LX)):adjustedJoystick(Math.abs(LX));
//        double t = RX < 0? -adjustedJoystick(Math.abs(RX)):adjustedJoystick(Math.abs(RX));

        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (-LY * PowerBase + LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double backLeftPower = (-LY * PowerBase - LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double frontRightPower = (-LY * PowerBase - LX * PowerStrafe - RX * PowerBaseTurn) / denominator;
        double backRightPower = (-LY * PowerBase + LX * PowerStrafe - RX * PowerBaseTurn) / denominator;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
        BL.setPower(backLeftPower);
    }

    public void motorDirectionTest(double FL, double FR, double BL, double BR){
        this.FL.setPower(FL);
        this.BL.setPower(BL);
        this.FR.setPower(FR);
        this.BR.setPower(BR);
    }
    public void runtoPositionTest(int Position){
        FL.setTargetPosition(Position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(1);
    }
    public void PowerToggle(boolean toggle) { // toggle code for a slow drive mode for fine adjustment
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {

                    PowerStrafe = 0;
                } else {
                    //edit these values to change drivecode

                    PowerStrafe = 1.05;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }
}
