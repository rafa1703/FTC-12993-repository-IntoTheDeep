package org.firstinspires.ftc.teamcode.system.accessory;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTime { // Do you need to cache 2 variables instead of just time.reset()
    double dt;
    double prev_time;
    double hz;
    Telemetry telemetry;
    public LoopTime(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
    public LoopTime()
    {
    }
    public void updateLoopTime (@NonNull Telemetry telemetry){

        dt = System.currentTimeMillis() - prev_time;
        telemetry.addData("Loop Time", dt / 1e6);
        prev_time = System.currentTimeMillis(); // this should possibly be run at the end of the loop to reset the time, might not work all packaged together
    }
    public void updateLoopTime (){

        dt = System.currentTimeMillis() - prev_time;
        telemetry.addData("Loop Time", dt / 1e6);
        prev_time = System.currentTimeMillis(); // this should possibly be run at the end of the loop to reset the time, might not work all packaged together
    }
}
