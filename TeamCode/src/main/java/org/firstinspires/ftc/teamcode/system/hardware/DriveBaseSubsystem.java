package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;

@Config // Allows dashboard to tune
public class DriveBaseSubsystem
{  // no constructor for this class

    public MotorPika
            FL,
            FR,
            BL,
            BR,
            hangMotor;

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
    int hangUpPos, hangDownPos;
    public enum HangState
    {
        READY,
        OUT,
        UP
    }
    Telemetry telemetry;
    public DriveBaseSubsystem(GeneralHardware hardware)
    {
        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;
        hangMotor = hardware.climbM;
        drivebaseSetup(true); // this has to be true for GVF, as we do glinding vectors
    }
    // this could be run in robothardware
    public void drivebaseSetup(boolean Float)
    {
        // zero brake behavior means when motors aren't powered, they will auto brake
        if (Float) setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        else setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour)
    {
        FR.setZeroPowerBehavior(zeroPowerBehaviour);
        BR.setZeroPowerBehavior(zeroPowerBehaviour);
        BL.setZeroPowerBehavior(zeroPowerBehaviour);
        FL.setZeroPowerBehavior(zeroPowerBehaviour);
    }
    public void HangState(HangState state)
    {
        switch (state)
        {
            case READY:
                if (Math.abs(hangMotor.getCurrentPosition() - hangDownPos) < 5)
                {
                    hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hangMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
                    hangMotor.setPower(0); // cut power
                }
                else
                {
                    hangMotor.setTargetPosition(hangDownPos);
                    hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hangMotor.setPower(1);
                }
                break;
            case OUT:
                hangMotor.setTargetPosition(hangDownPos);
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor.setPower(1);
                break;
            case UP:
                hangMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
                hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hangMotor.setPower(-1);
                break;
        }
    }
    public static double adjustedJoystick(double x) {
        double y = Math.pow(c-x,m);
        return Math.pow(x,y);
    }

    public void drive(double LY, double LX, double RX) {
        double f = LY < 0? -adjustedJoystick(Math.abs(LY)):adjustedJoystick(Math.abs(LY));
        double s = LX < 0? -adjustedJoystick(Math.abs(LX)):adjustedJoystick(Math.abs(LX));
        double t = RX < 0? -adjustedJoystick(Math.abs(RX)):adjustedJoystick(Math.abs(RX));

        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (-f*PowerBase + s*PowerStrafe + t*PowerBaseTurn) / denominator;
        double backLeftPower = (-f*PowerBase - s*PowerStrafe + t*PowerBaseTurn) / denominator;
        double frontRightPower = (-f*PowerBase - s*PowerStrafe - t*PowerBaseTurn) / denominator;
        double backRightPower = (-f*PowerBase + s*PowerStrafe - t*PowerBaseTurn) / denominator;

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
