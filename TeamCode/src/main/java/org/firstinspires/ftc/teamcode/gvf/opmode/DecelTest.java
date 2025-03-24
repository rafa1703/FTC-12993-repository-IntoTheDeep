package org.firstinspires.ftc.teamcode.gvf.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;


@Config
@TeleOp(name = "Deceleration Tuner", group="Test")
public class DecelTest extends LinearOpMode
{

    FtcDashboard dash;

    GeneralHardware hardware;

    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 1;
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        waitForStart();
        hardware.drive.getLocalizer().setPose(new Pose(0, 0 , Math.toRadians(0)));

        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested())
        {
            hardware.update();
            switch (step)
            {
                case 0:
                    if (timer.seconds() <= accelerationTime)
                    {
                        hardware.drive.setTargetVector(new Vector(0, 1));
                    } else
                    {
                        step++;
                        timer.reset();
                        velocityAtStop = hardware.drive.getLocalizer().getVelocity().getY();
                        hardware.drive.setTargetVector(new Vector());
                    }
                    break;
                case 1:
                    if (hardware.drive.getLocalizer().getVelocity().getMagnitude() <= 0.1)
                    {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = velocityAtStop / deltaTime;
                        stopped = hardware.drive.stopped();
                    }
                    break;
            }
            // 2.62 y decel, 87.68 x decel

            hardware.drive.update();
            telemetry.addData("pose", hardware.drive.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Deceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", hardware.drive.getLocalizer().getVelocity().getX());
            telemetry.addData("Velocity y", hardware.drive.getLocalizer().getVelocity().getY());
            telemetry.addData("Imu angle", hardware.drive.getLocalizer().getHeading());

            telemetry.update();
        }
    }
}
