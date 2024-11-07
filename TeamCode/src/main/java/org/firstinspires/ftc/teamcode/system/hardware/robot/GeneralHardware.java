package org.firstinspires.ftc.teamcode.system.hardware.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import org.firstinspires.ftc.teamcode.gvf.Localizer;
import org.firstinspires.ftc.teamcode.gvf.LocalizerCustomVel;
import org.firstinspires.ftc.teamcode.gvf.LocalizerOTOS;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;


public class GeneralHardware
{
    public HardwareMap hardwareMap;

    public DcMotorEx intakeM, FR, intakeSlidesM, BR;
    public DcMotorEx FL, BL, outtakeLiftM, meh3;

    public Encoder perpendicularOdo, ech1, ech2, parallelOdo;
    public Encoder eeh0, eeh1, eeh2, eeh3;

    public ServoImplEx chuteS, clipS, intakeLeftArmS, intakeRightArmS, outtakeLeftArmS, outtakeRightArmS;
    public ServoImplEx clawS, pivotS, flapS, seh3, seh4, seh5;
    public RevColorSensorV3 cs0;
    public ImuThread imu;

    public LocalizerCustomVel localizer;
    public LocalizerOTOS otosLocalizer;
    public GoBildaPinpointDriver pinpointLocalizer;
    public MecanumDrive drive;

    public VoltageSensor voltageSensor;
    public TimedSupplier<Double> voltageSupplier;
    public static double voltage;

    public DigitalChannel dc0, dc1;
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

        intakeM = hm.get(DcMotorEx.class, "Intake");
        FR = hm.get(DcMotorEx.class, "FR");
        intakeSlidesM = hm.get(DcMotorEx.class, "IntakeSlides");
        BR = hm.get(DcMotorEx.class, "BR");

        FL = hm.get(DcMotorEx.class, "FL");
        BL = hm.get(DcMotorEx.class, "BL");
        outtakeLiftM = hm.get(DcMotorEx.class, "OuttakeSlides");
        //meh3 = hm.get(DcMotorEx.class, "eh3");

        perpendicularOdo = new Encoder(intakeM);
        ech1 = new Encoder(FR);
        ech2 = new Encoder(intakeSlidesM);
        parallelOdo = new Encoder(BR);

        eeh0 = new Encoder(FL);
        eeh1 = new Encoder(BL);
        eeh2 = new Encoder(outtakeLiftM);
        //eeh3 = new Encoder(meh3);


        chuteS = hm.get(ServoImplEx.class, "chuteS");
        clipS = hm.get(ServoImplEx.class, "clipS");
        intakeLeftArmS = hm.get(ServoImplEx.class, "intakeLeftArmS");
        intakeRightArmS = hm.get(ServoImplEx.class, "intakeRightArmS");
        outtakeLeftArmS = hm.get(ServoImplEx.class, "outtakeLeftArmS");
        outtakeRightArmS = hm.get(ServoImplEx.class, "outtakeRightArmS");

        clawS = hm.get(ServoImplEx.class, "clawS");
        pivotS = hm.get(ServoImplEx.class, "pivotS");
        flapS = hm.get(ServoImplEx.class, "flapS");
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
            //TODO: reverse odo encoder
            //parallelOdo.setDirection(REVERSE)
            // TODO: change here depending on odometry type
            imu = new ImuThread(hm);
            imu.initImuThread();
            localizer = new LocalizerCustomVel(this);

            //otosLocalizer = new LocalizerOTOS(this);

//            pinpointLocalizer = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//            pinpointLocalizer.setOffsets(0, 0);
//            pinpointLocalizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//            pinpointLocalizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//            pinpointLocalizer.resetPosAndIMU();

            MecanumDrive.headingMultiplier = 1;
            drive = new MecanumDrive(this, MecanumDrive.RunMode.Vector);
            drive.setRunMode(MecanumDrive.RunMode.Vector);

            S = side == Side.Red ? 1 : -1;
            A = side == Side.Red ? Math.toRadians(0) : Math.toRadians(180);
        }
    }
    /** This has to be called when the opMode initializes, only auto **/
    public void startThreads(LinearOpMode opMode)
    {
        imu.startThread(opMode);
    }
    // this is only for auto, i want to only have to call a single update method inside the opMode
    public void update()
    {
        resetCacheHubs();
        voltage = voltageSupplier.get();
        drive.update();
        //localizer.update();
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