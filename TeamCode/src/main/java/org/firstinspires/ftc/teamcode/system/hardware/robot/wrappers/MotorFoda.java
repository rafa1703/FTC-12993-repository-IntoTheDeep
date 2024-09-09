package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PIDR;

public class MotorFoda implements IHardware
{
    public boolean ENABLED = true;
    private final DcMotorEx motor;
    private double power, prevPower;
    private final Object lock = new Object();
    private final double EPSILON_DELTA = 0.005; // Voltage
    private final double REACHED_THRESHOLD = 15; // TICKS
    private boolean async = false;
    private double target;
    private double position;

    private PIDR pid;
    private double p, i, d, f;
    private boolean PID = false;
    private boolean write = true;
    private boolean read = true;
    public MotorFoda(DcMotorEx motor)
    {
        this(motor, 0, 0, 0, 0);
    }
    public MotorFoda(DcMotorEx motor, double p, double i, double d, double f)
    {
        this.motor = motor;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        pid = new PIDR(p, i, d, f);
    }
    public MotorFoda(DcMotorEx motor, double p, double i, double d)
    {
        this(motor, p, i, d, 0);
    }
    public void setUp(boolean read, boolean write, boolean async)
    {
        this.read = read;
        this.write = write;
        this.async = async;
    }

    public void updatePower()
    {
        if (
                (Math.abs(power - prevPower) > EPSILON_DELTA) ||
                (power == 0.0 && prevPower != 0.0) ||
                (power >= 1.0 && !(prevPower >= 1.0)) ||
                (power <= -1.0 && !(prevPower <= -1.0))
        )
        {
            prevPower = power;
            write = true;
        }
        else write = false;
    }

    public void setPower(double power)
    {
        this.power = power;
    }
    public void setPowerAsync()
    {
        synchronized (lock)
        {
            motor.setPower(power);
        }
    }
    public void getPositionAsync()
    {
        synchronized (lock)
        {
            position = motor.getCurrentPosition();
        }
    }
    public double getPosition()
    {
        return position;
    }
    public void runPID()
    {
        power = pid.update(target, position, 1);
    }
    public boolean reached()
    {
        return Math.abs(target - position) < REACHED_THRESHOLD;
    }

    public void update()
    {
        writes();
        reads();
    }

    @Override
    public void reads()
    {
        if (!ENABLED) return;
        if (read)
        {
            if (async) getPositionAsync();
            else motor.getCurrentPosition();
        }
    }

    @Override
    public void writes()
    {
        if (!ENABLED) return;
        if (PID) runPID();
        updatePower();
        if (write)
        {
            if (async) setPowerAsync();
            else motor.setPower(power);
        }
    }

    @Override
    public void setPowers(double power)
    {
        setPower(power);
    }

    public void setENABLED(boolean ENABLED)
    {
        this.ENABLED = ENABLED;
    }

    public void setWrite(boolean write)
    {
        this.write = write;
    }

    public void setRead(boolean read)
    {
        this.read = read;
    }
    public void setAsync(boolean async)
    {
        this.async = async;
    }
    public void setTarget(double target)
    {
        this.target = target;
    }

    public double getPower()
    {
        return power;
    }

    public void setPID(boolean PID)
    {
        this.PID = PID;
    }

    public double getTarget()
    {
        return target;
    }

}
