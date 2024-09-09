package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoFoda implements IHardware
{
    public boolean ENABLED = true;
    ServoImplEx servo;
    double prevPos, pos, targetPos;
    private final Object lock = new Object();
    private final double EPSILON_DELTA = 0.005; // ticks
    private boolean async = false;
    private boolean write = true;
    private boolean profiled;

    public ServoFoda(ServoImplEx servo)
    {
        this.servo = servo;
    }
    public void setUp(boolean write, boolean async)
    {
        this.write = write;
        this.async = async;
    }
    public void updatePos()
    {
        if (
                (Math.abs(pos - prevPos) > EPSILON_DELTA) ||
                        (pos == 0.0 && prevPos != 0.0) ||
                        (pos >= 255.0 && !(prevPos >= 255.0)) ||
                        (pos <= -255.0 && !(prevPos <= -255.0))
        )
        {
            prevPos = pos;
            write = true;
        }
        else write = false;
    }
    private void setPosAsync()
    {
        synchronized (lock)
        {
            servo.setPosition(pos);
        }
    }
    private void setPosition()
    {
        if (profiled) servo.setPosition(targetPos);
        servo.setPosition(pos);
    }
    public void runProfiled()
    {
        targetPos = interpolation(prevPos, pos, Math.abs(prevPos - pos) / 20);
    }
    @Override
    public void update()
    {
        //reads();
        writes();
    }

    @Override
    public void reads()
    {
        // there is technically no reads in a servo
    }

    @Override
    public void writes()
    {
//        if (profiled) runProfiled();
        updatePos();
        if (write)
        {
            if (async) setPosAsync();
            else servo.setPosition(pos);
        }
    }
    private double interpolation(double p1, double p2, double t) {
        return (1 - t) * p1 + t * p2;
    }
    public double getPos()
    {
        return pos;
    }

    public void setPos(double pos)
    {
        this.pos = pos;
    }
}
