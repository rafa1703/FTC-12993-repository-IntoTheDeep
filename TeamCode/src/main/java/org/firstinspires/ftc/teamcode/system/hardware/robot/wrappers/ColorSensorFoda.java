package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;

public class ColorSensorFoda implements IHardware
{
    public boolean ENABLED = true;
    ColorSensor colorSensor;
    int value;
    TimedSupplier<Integer> supplier;
    double period;

    public ColorSensorFoda(ColorSensor colorSensor, double period)
    {
        this.colorSensor = colorSensor;
        this.period = period;
        init(true, false,false,false,false);
    }
    public void init(boolean alpha, boolean blue, boolean red, boolean green, boolean argb)
    {
        if (alpha)
        {
            supplier = new TimedSupplier<>(colorSensor::alpha, period);
            return;
        }
        if (blue)
        {
            supplier = new TimedSupplier<>(colorSensor::blue, period);
            return;
        }
        if (red)
        {
            supplier = new TimedSupplier<>(colorSensor::red, period);
            return;
        }
        if (green)
        {
            supplier = new TimedSupplier<>(colorSensor::green, period);
            return;
        }
        if (argb)
        {
            supplier = new TimedSupplier<>(colorSensor::argb, period);
        }
    }
    @Override
    public void update()
    {

    }

    @Override
    public void reads()
    {
        value = supplier.get();
    }

    @Override
    public void writes()
    {

    }
    public int getValue()
    {
        return value;
    }
}
