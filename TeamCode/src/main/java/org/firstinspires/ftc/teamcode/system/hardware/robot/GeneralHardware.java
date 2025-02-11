package org.firstinspires.ftc.teamcode.system.hardware.robot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import org.firstinspires.ftc.teamcode.gvf.LocalizerPinpoint;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.CRServoPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;


public class GeneralHardware
{
    public HardwareMap hardwareMap;

    public MotorPika FR, intakeSlidesM, BR;
    public MotorPika FL, BL, outtakeLiftM, turretM;
    public AnalogInput turretEncoder;
    public Encoder perpendicularOdo, parallelOdo;
    public ServoPika turretS, clipS, intakeArmS, leftPTOS, rightPTOS;
    public CRServoPika intakeS;
    public ServoPika clawS, wristS, pivotS, armS;
    public RevColorSensorV3 colourSensor;
    public DistanceSensor distanceSensor;
    public GoBildaPinpointDriver pinpoint;
    public LocalizerPinpoint localizerPinpoint;

    public MecanumDrive drive;

    public VoltageSensor voltageSensor;
    public TimedSupplier<Double> imuSupplier;
    public IMU imu;
    public TimedSupplier<Double> voltageSupplier;
    public static double voltage;
    public Limelight3A limelight;

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

        turretM = new MotorPika(hm.get(DcMotorEx.class, "Turret"));
        FR = new MotorPika(hm.get(DcMotorEx.class, "FR"));
        intakeSlidesM = new MotorPika(hm.get(DcMotorEx.class, "IntakeSlides"));
        BR = new MotorPika(hm.get(DcMotorEx.class, "BR"));

        FL = new MotorPika(hm.get(DcMotorEx.class, "FL"));
        BL = new MotorPika(hm.get(DcMotorEx.class, "BL"));
        outtakeLiftM = new MotorPika(hm.get(DcMotorEx.class, "OuttakeSlides"));

       /* perpendicularOdo = new Encoder(intakeM.getMotor());
        parallelOdo = new Encoder(BR.getMotor());*/

        intakeS = new CRServoPika(hm.get(CRServoImplEx.class, "intakeS"));
        clipS = new ServoPika(hm.get(ServoImplEx.class, "clipS"));
        intakeArmS = new ServoPika(hm.get(ServoImplEx.class, "intakeS"));
        leftPTOS = new ServoPika(hm.get(ServoImplEx.class, "leftPTOS"));
        rightPTOS = new ServoPika(hm.get(ServoImplEx.class, "rightPTOS"));
        turretS = new ServoPika(hm.get(ServoImplEx.class, "turretS"));

        clawS = new ServoPika(hm.get(ServoImplEx.class, "clawS"));
        wristS = new ServoPika(hm.get(ServoImplEx.class, "wristS"));
        armS = new ServoPika(hm.get(ServoImplEx.class, "armS"));
        pivotS = new ServoPika(hm.get(ServoImplEx.class, "flapS"));
        //seh3 = hm.get(ServoImplEx.class, "seh3");
        //seh4 = hm.get(ServoImplEx.class, "seh4");
        //seh5 = hm.get(ServoImplEx.class, "seh5");

        turretEncoder = hm.get(AnalogInput.class, "turretEncoder");
        colourSensor = hm.get(RevColorSensorV3.class, "colorSensor");
        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 40);
        //dc0 = hm.get(DigitalChannel.class, "dc0");
        //dc1 = hm.get(DigitalChannel.class, "dc1");


        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (auto)
        {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            //TODO: here we change the wheels offsets
            // length 13.8 (0.5 = 6.9)
            // y pod 6.9 - 3.5 = 3.4
            // width 14.5 (0.5 = 7.25)
            // x pod 7.25 - 3.35 = 3.9
            pinpoint.setOffsets(-100.33, 89);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
            localizerPinpoint = new LocalizerPinpoint(this);
            MecanumDrive.headingMultiplier = 1;
            drive = new MecanumDrive(this, MecanumDrive.RunMode.Vector);
            drive.setRunMode(MecanumDrive.RunMode.Vector);

            limelight = hardwareMap.get(Limelight3A.class, "Limelight");

            S = side == Side.Red ? 1 : -1;
            A = side == Side.Red ? Math.toRadians(0) : Math.toRadians(180);
        }
        else imu = hardwareMap.get(BHI260IMU.class, "imu"); // we initialize a normal instance of the imu for tele
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