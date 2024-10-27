
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class redCloseAuto extends LinearOpMode
{

    enum autoState {
        INTAKE,
        DEPOSIT,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.DEPOSIT;
    GeneralHardware hardware;
    Paths trajectories = new Paths();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.startThreads(this);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        while (!isStarted())
        {
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
        }
        waitForStart();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            autoSequence();
            hardware.update();
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case INTAKE:
                if(true)
                {
                    state = autoState.DEPOSIT;
                    resetTimer();
                }

                break;
            case DEPOSIT:

                break;
            case PARK:

                break;
            case IDLE:
                break;
        }
    }
    public void outtakeLiftPresets(boolean isSample, boolean isLow, int offSet)
    {
        if (isSample)
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos + offSet);
        }
        else
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos + offSet);
        }
    }
    public boolean delay(double delayTime)
    {
        return (globalTimer - sequenceTimer) > delayTime;
    }

    public void resetTimer()
    {
        sequenceTimer = globalTimer;
    }
}
