package org.firstinspires.ftc.teamcode.system.hardware.robot;

import androidx.annotation.GuardedBy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.IHardware;

import java.util.ArrayList;

public class WriteThread
{
    private final Object lock = new Object();
    @GuardedBy("lock")
    Thread thread;
    ArrayList<IHardware> list = new ArrayList<>();
    public WriteThread(ArrayList<IHardware> list)
    {
        this.list = list;
    }
    public WriteThread(IHardware hardware)
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
                        hardware.writes();
                    }
                }
            }
        });
        thread.start();
    }

}
