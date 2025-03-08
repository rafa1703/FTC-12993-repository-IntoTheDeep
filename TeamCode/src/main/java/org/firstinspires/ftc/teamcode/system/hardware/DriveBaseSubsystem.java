package org.firstinspires.ftc.teamcode.system.hardware;

import static com.sun.tools.doclint.Entity.le;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

@Config // Allows dashboard to tune
public class DriveBaseSubsystem
{  // no constructor for this class

    public MotorPika
            FL,
            FR,
            BL,
            BR;
    ServoPika
        ptoS;


    public enum PTOState
    {
        OUT(0.23),
        IN(0);

        public final double pos;

        PTOState(double pos)
        {
            this.pos = pos;
        }
    }
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

    Telemetry telemetry;
    public DriveBaseSubsystem(GeneralHardware hardware)
    {
        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;
        ptoS = hardware.ptoS;
        drivebaseSetup(true); // this has to be true for GVF, as we do glinding vectors
    }
    public DriveBaseSubsystem(HardwareMap hardwareMap)
    {
        FL = new MotorPika(hardwareMap.get(DcMotorEx.class, "FL"));
        FR = new MotorPika(hardwareMap.get(DcMotorEx.class, "FR"));
        BL = new MotorPika(hardwareMap.get(DcMotorEx.class, "BL"));
        BR = new MotorPika(hardwareMap.get(DcMotorEx.class, "BR"));

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        drivebaseSetup(true); // this has to be true for GVF, as we do glinding vectors
    }
    // this could be run in robothardware
    public void drivebaseSetup(boolean Float)
    {
        // zero brake behavior means when motors aren't powered, they will auto brake
        if (Float) setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        else setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour)
    {
        FR.setZeroPowerBehavior(zeroPowerBehaviour);
        BR.setZeroPowerBehavior(zeroPowerBehaviour);
        BL.setZeroPowerBehavior(zeroPowerBehaviour);
        FL.setZeroPowerBehavior(zeroPowerBehaviour);
    }
    public void PTOState(PTOState state)
    {
        ptoS.setPosition(state.pos);
    }
    public void PTOSetPosition(double pos)
    {
        ptoS.setPosition(pos);
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
    public void ptoMotorsSetPower(double pow){
        this.BL.setPower(pow);
        this.BR.setPower(pow);
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
