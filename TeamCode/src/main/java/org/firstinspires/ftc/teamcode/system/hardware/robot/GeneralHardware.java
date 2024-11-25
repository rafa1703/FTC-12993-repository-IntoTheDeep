package org.firstinspires.ftc.teamcode.system.hardware.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import org.firstinspires.ftc.teamcode.gvf.LocalizerPinpoint;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;


public class GeneralHardware
{
    public HardwareMap hardwareMap;

    public MotorPika intakeM, FR, intakeSlidesM, BR;
    public MotorPika FL, BL, outtakeLiftM, hangM;

    public Encoder perpendicularOdo, parallelOdo;
    public ServoPika chuteS, clipS, intakeLeftArmS, intakeRightArmS, outtakeLeftArmS, outtakeRightArmS;
    public ServoPika clawS, wristS, flapS, seh3, seh4, seh5;
    public RevColorSensorV3 cs0;
    public GoBildaPinpointDriver pinpoint;
    public LocalizerPinpoint localizerPinpoint;

    public MecanumDrive drive;

    public VoltageSensor voltageSensor;
    public TimedSupplier<Double> voltageSupplier;
    public static double voltage;
    public ColorSensor csO;

    public enum Side
    {
        Red, Blue
    }
    public final Side side;
    public static double S, A; // s is the side multiplier and a is the angle multiplier

    LynxModule chub, ehub;

    public GeneralHardware(HardwareMap hm, Side side)
    {
        this(hm, side, false);
    }
    public GeneralHardware(HardwareMap hm, Side side, boolean auto)
    {
        this.hardwareMap = hm;
        this.side = side;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class))
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() &&
                    LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) chub = hub;
            else ehub = hub;
        }

        intakeM = new MotorPika(hm.get(DcMotorEx.class, "Intake"));
        FR = new MotorPika(hm.get(DcMotorEx.class, "FR"));
        intakeSlidesM = new MotorPika(hm.get(DcMotorEx.class, "IntakeSlides"));
        BR = new MotorPika(hm.get(DcMotorEx.class, "BR"));

        FL = new MotorPika(hm.get(DcMotorEx.class, "FL"));
        BL = new MotorPika(hm.get(DcMotorEx.class, "BL"));
        outtakeLiftM = new MotorPika(hm.get(DcMotorEx.class, "OuttakeSlides"));
        hangM = new MotorPika(hm.get(DcMotorEx.class, "Hang"));

       /* perpendicularOdo = new Encoder(intakeM.getMotor());
        parallelOdo = new Encoder(BR.getMotor());*/

        chuteS = new ServoPika(hm.get(ServoImplEx.class, "chuteS"));
        clipS = new ServoPika(hm.get(ServoImplEx.class, "clipS"));
        intakeLeftArmS = new ServoPika(hm.get(ServoImplEx.class, "intakeLeftArmS"));
        intakeRightArmS = new ServoPika(hm.get(ServoImplEx.class, "intakeRightArmS"));
        outtakeLeftArmS = new ServoPika(hm.get(ServoImplEx.class, "outtakeLeftArmS"));
        outtakeRightArmS = new ServoPika(hm.get(ServoImplEx.class, "outtakeRightArmS"));

        clawS = new ServoPika(hm.get(ServoImplEx.class, "clawS"));
        wristS = new ServoPika(hm.get(ServoImplEx.class, "pivotS"));
        flapS = new ServoPika(hm.get(ServoImplEx.class, "flapS"));
        //seh3 = hm.get(ServoImplEx.class, "seh3");
        //seh4 = hm.get(ServoImplEx.class, "seh4");
        //seh5 = hm.get(ServoImplEx.class, "seh5");

        cs0 = hm.get(RevColorSensorV3.class, "colorSensor");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 40);
        //dc0 = hm.get(DigitalChannel.class, "dc0");
        //dc1 = hm.get(DigitalChannel.class, "dc1");


        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (auto)
        {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            //TODO: here we change the wheels offsets
            pinpoint.setOffsets(0, 0);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
            localizerPinpoint = new LocalizerPinpoint(this);
            MecanumDrive.headingMultiplier = 1;
            drive = new MecanumDrive(this, MecanumDrive.RunMode.Vector);
            drive.setRunMode(MecanumDrive.RunMode.Vector);

            S = side == Side.Red ? 1 : -1;
            A = side == Side.Red ? Math.toRadians(0) : Math.toRadians(180);
        }
    }
    @Deprecated
    /** This has to be called when the opMode initializes, only auto **/
    public void startThreads(LinearOpMode opMode)
    {

    }
    // this is only for auto, i want to only have to call a single update method inside the opMode
    public void update()
    {
        resetCacheHubs();
        voltage = voltageSupplier.get();
        drive.update();
    }
    public void resetCacheHubs()
    {
        chub.clearBulkCache();
        ehub.clearBulkCache();
    }

    public double getVoltage()
    {
        voltage = voltageSupplier.get();
        return voltage;
    }
}