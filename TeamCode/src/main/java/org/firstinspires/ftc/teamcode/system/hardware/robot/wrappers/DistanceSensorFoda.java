package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;

public class DistanceSensorFoda implements IHardware
{
    public boolean ENABLED = true;
    private DistanceSensor distanceSensor;
    private TimedSupplier<Double> supplier;
    private double distance;
    public DistanceSensorFoda(DistanceSensor distanceSensor, double period)
    {
        this.distanceSensor = distanceSensor;
        supplier = new TimedSupplier<>(() -> distanceSensor.getDistance(DistanceUnit.CM), period);
    }

    @Override
    public void update()
    {

    }

    @Override
    public void reads()
    {
        distance = supplier.get();
    }

    @Override
    public void writes()
    {

    }

    public double getDistance()
    {
        return distance;
    }
}
