package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class TeleDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    enum OuttakeState {
        READY,
        INTAKE_EXTENDO,
        INTAKE_EXTENDO_DROP,
        INTAKE,
        INTAKE_DROP,
        INTAKE_TO_TRANSFER,
        TRANSFER_START,
        TRANSFER_END,
        OUTTAKE_ADJUST,
        DEPOSIT,
        RETURN,
        MANUAL_ENCODER_RESET,
        IDLE
    }
    OuttakeState state = OuttakeState.READY;
    double colorValue;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, false);
        GlobalTimer = new ElapsedTime(System.currentTimeMillis());
        sequenceTimer = globalTimer;
        intakeSubsystem = new IntakeSubsystem(hardware);
        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            intakeSubsystem.intakeReads(state == OuttakeState.INTAKE || state == OuttakeState.INTAKE_DROP || state == OuttakeState.INTAKE_EXTENDO);
            outtakeSequence();
        }
    }

    public void outtakeSequence()
    {
        switch (state)
        {
            case READY:
                if (delay(100))
                {
                    if (gamepad1.right_bumper)
                    {
                        state = OuttakeState.INTAKE;
                        resetTimer();
                    }
                    else if (gamepad1.left_bumper)
                    {
                        state =  OuttakeState.INTAKE_EXTENDO;
                        resetTimer();
                    }
                }
                readyOuttake();
                intakeArmHeight();
                if (gamepad1.b || gamepad2.b) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);

                break;
            case INTAKE_EXTENDO:
                break;
            case INTAKE:
                if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                }
                intakeSubsystem.intakeChuteState(IntakeSubsystem.IntakeChuteServoState.UP);
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.DOWN);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                // whenever we want to intake the intake goes down but with this the driver can hold the button to control it
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();
                if (colorValue > 1000)
                {
                   if (!colorLogic(colorValue)) // if we picked the wrong color we initialize the drop sequence
                   {
                       state = OuttakeState.INTAKE_DROP;
                       resetTimer();
                   }
                   else
                   {
                       state = OuttakeState.INTAKE_TO_TRANSFER;
                       gamepad1.rumbleBlips(2);
                       resetTimer();
                   }
                }
                break;
            case INTAKE_DROP:
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                if (colorValue < 1000 && delay(200))
                {
                    state = OuttakeState.INTAKE;
                    resetTimer();
                }
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drops
                if (delay(50))
                {
                    intakeSubsystem.intakeChuteState(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    // this assumes that the sample is stuck in the chute so we spin the intake
                }
                break;
            case INTAKE_TO_TRANSFER:
                break;
            case TRANSFER_START:
                break;
            case TRANSFER_END:
                break;
            case OUTTAKE_ADJUST:
                break;
            case DEPOSIT:
                break;
            case RETURN:
                break;
            case MANUAL_ENCODER_RESET:
                break;
            case IDLE:
                break;
        }
        // we run everything here that isn't state specific
        if ((gamepad2.b || gamepad1.b) && (state != OuttakeState.READY) && (state != OuttakeState.MANUAL_ENCODER_RESET))
        {
            // can't reset if in manual reset lmao
            state = OuttakeState.RETURN; // if b is pressed at any state then return to ready
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

    public void intakeArmHeight()
    {
        if (gamepad1.x || gamepad2.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
        else if (gamepad1.a || gamepad2.a) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
    }

    public boolean colorLogic(double colorValue)
    {
        if (colorValue > 5000) // assuming 5000 is red threshold
        {
            return (intakeSubsystem.getSide() == GeneralHardware.Side.Red);
        }
        if (colorValue < 3000) // assuming 3000 is yellow threshold
        {
            return (intakeSubsystem.getSide() == GeneralHardware.Side.Blue);
        }
        else return true; // this means is a yellow (3000-5000)
    }

    public void readyOuttake()
    {
        intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.DOWN);
        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
    }
}
