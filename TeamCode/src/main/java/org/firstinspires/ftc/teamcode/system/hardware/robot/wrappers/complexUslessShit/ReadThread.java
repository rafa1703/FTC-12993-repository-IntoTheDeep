package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.complexUslessShit;

import androidx.annotation.GuardedBy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class ReadThread
{
    private final Object lock = new Object();
    @GuardedBy("lock")
    Thread thread;
    ArrayList<IHardware> list = new ArrayList<>();
    public ReadThread(ArrayList<IHardware> list)
    {
        this.list = list;
    }
    public ReadThread(IHardware hardware)
    {
        list.add(hardware);
    }
    public void startThread(LinearOpMode opMode)
    {
        // lambda method run from Thread
        thread = new Thread(() -> {
            while (!opMode.isStopRequested())
            {
                synchronized (lock)
                {
                    for (IHardware hardware :
                            list)
                    {
                        hardware.reads();
                    }
                }
            }
        });
        thread.start();
    }
}
