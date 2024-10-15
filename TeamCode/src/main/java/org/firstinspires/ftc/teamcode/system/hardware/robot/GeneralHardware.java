package org.firstinspires.ftc.teamcode.system.hardware.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import org.firstinspires.ftc.teamcode.gvf.Localizer;
import org.firstinspires.ftc.teamcode.gvf.LocalizerOTOS;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;


public class GeneralHardware
{
    public HardwareMap hardwareMap;

    public DcMotorEx mch0, mch1, mch2, mch3;
    public DcMotorEx meh0, meh1, meh2, meh3;

    public Encoder ech0, ech1, ech2, ech3;
    public Encoder eeh0, eeh1, eeh2, eeh3;

    public ServoImplEx sch0, sch1, sch2, sch3, sch4, sch5;
    public ServoImplEx seh0, seh1, seh2, seh3, seh4, seh5;
    public ColorSensor cs0;
    public ImuThread imu;

    public Localizer localizer;
    public LocalizerOTOS otosLocalizer;
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

        mch0 = hm.get(DcMotorEx.class, "FR");
        mch1 = hm.get(DcMotorEx.class, "ch1");
        mch2 = hm.get(DcMotorEx.class, "ch2");
        mch3 = hm.get(DcMotorEx.class, "BR");

        meh0 = hm.get(DcMotorEx.class, "eh0");
        meh1 = hm.get(DcMotorEx.class, "eh1");
        meh2 = hm.get(DcMotorEx.class, "eh2");
        meh3 = hm.get(DcMotorEx.class, "eh3");

        ech0 = new Encoder(mch0);
        ech1 = new Encoder(mch1);
        ech2 = new Encoder(mch2);
        ech3 = new Encoder(mch3);

        eeh0 = new Encoder(meh0);
        eeh1 = new Encoder(meh1);
        eeh2 = new Encoder(meh2);
        eeh3 = new Encoder(meh3);


        sch0 = hm.get(ServoImplEx.class, "sch0");
        sch1 = hm.get(ServoImplEx.class, "sch1");
        sch2 = hm.get(ServoImplEx.class, "sch2");
        sch3 = hm.get(ServoImplEx.class, "sch3");
        sch4 = hm.get(ServoImplEx.class, "sch4");
        sch5 = hm.get(ServoImplEx.class, "sch5");

        seh0 = hm.get(ServoImplEx.class, "seh0");
        seh1 = hm.get(ServoImplEx.class, "seh1");
        seh2 = hm.get(ServoImplEx.class, "seh2");
        seh3 = hm.get(ServoImplEx.class, "seh3");
        seh4 = hm.get(ServoImplEx.class, "seh4");
        seh5 = hm.get(ServoImplEx.class, "seh5");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 40);
        dc0 = hm.get(DigitalChannel.class, "dc0");
        dc1 = hm.get(DigitalChannel.class, "dc1");

        if (auto)
        {
            // TODO: change here depending on odometry type
            imu = new ImuThread(hm);
            imu.initImuThread();
            localizer = new Localizer(this);
            //otosLocalizer = new LocalizerOTOS(this);

            MecanumDrive.headingMultiplier = 1;
            drive = new MecanumDrive(this, MecanumDrive.RunMode.Vector);

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
        //localizer.update();
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