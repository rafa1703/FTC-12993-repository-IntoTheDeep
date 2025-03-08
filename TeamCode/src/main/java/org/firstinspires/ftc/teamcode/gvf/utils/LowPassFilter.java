package org.firstinspires.ftc.teamcode.gvf.utils;

public class LowPassFilter {

    private final double t;
    private double lastValue;

    public LowPassFilter(double t, double initialValue){
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double v){
        double newValue = lastValue + t * (v - lastValue);
        this.lastValue = newValue;
        return newValue;
    }
}