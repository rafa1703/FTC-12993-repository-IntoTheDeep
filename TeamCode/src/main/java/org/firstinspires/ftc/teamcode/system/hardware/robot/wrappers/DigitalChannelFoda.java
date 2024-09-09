package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;

public class DigitalChannelFoda implements IHardware
{
    public boolean ENABLED = true;
    DigitalChannel channel;
    boolean state;
    TimedSupplier<Boolean> supplier;

    public DigitalChannelFoda(DigitalChannel channel, long period)
    {
        this.channel = channel;
        supplier = new TimedSupplier<>(channel::getState, period);
    }

    @Override
    public void update()
    {
    }

    @Override
    public void reads()
    {
        state = supplier.get();
    }

    @Override
    public void writes()
    {
    }

    public boolean getState()
    {
        return state;
    }
}
