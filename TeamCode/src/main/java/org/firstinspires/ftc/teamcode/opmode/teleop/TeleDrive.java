package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class TeleDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    GeneralHardware hardware;
    enum OuttakeState {
        READY,
        INTAKE_EXTENDO,
        INTAKE_EXTENDO_DROP,
        INTAKE,
        INTAKE_DROP,
        TRANSFER_START,
        TRANSFER_END,
        OUTTAKE_ADJUST,
        DEPOSIT,
        SAMPLE_DEPOSIT,
        SPECIMEN_DEPOSIT,
        RETURN,
        MANUAL_ENCODER_RESET,
        IDLE
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    double colorValue;
    int intakeSlideTarget;
    int liftTarget;
    boolean isLow = false, isBucket = false; // lol this is gonna work this way and i hate it
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, false);
        GlobalTimer = new ElapsedTime(System.currentTimeMillis());
        sequenceTimer = globalTimer;
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        //telemetry.setMsTransmissionInterval(11);
        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            intakeSubsystem.intakeReads(state == OuttakeState.INTAKE || state == OuttakeState.INTAKE_DROP || state == OuttakeState.INTAKE_EXTENDO);
            outtakeSubsystem.outtakeReads();
            outtakeSequence();
            telemetry.update();
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
                    } else if (gamepad1.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO;
                        intakeSubsystem.intakeClipState(IntakeSubsystem.IntakeClipServoState.OPEN);
                        intakeSlideTarget = slideTeleClose;
                        intakeSlideBtn.upToggle(gamepad1.left_bumper);
                        resetTimer();
                    }
                }
                readyOuttake();
                intakeArmHeight();
                if (gamepad1.b || gamepad2.b)
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);

                break;
            case INTAKE_EXTENDO:
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();

                if (delay(90))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (delay(200) && colorValue > 1000)
                    {
                        if (!colorLogic(colorValue)) // if we picked the wrong color we initialize the drop sequence
                        {
                            state = OuttakeState.INTAKE_DROP;
                            resetTimer();
                        } else
                        {
                            state = OuttakeState.TRANSFER_START;
                            gamepad1.rumbleBlips(2);
                            resetTimer();
                        }
                    }
                }
                break;
            case INTAKE_EXTENDO_DROP:
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                if (colorValue < 1000 && delay(200))
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    resetTimer();
                }
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drops
                if (delay(50))
                {
                    intakeSubsystem.intakeChuteState(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    // this assumes that the sample is stuck in the chute so we spin the intake
                }
                break;
            case INTAKE:
                if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1.left_bumper);
                    resetTimer();
                }
                intakeSubsystem.intakeChuteState(IntakeSubsystem.IntakeChuteServoState.UP);
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.DOWN);
                // whenever we want to intake the intake goes down but with this the driver can hold the button to control it
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();
                if (delay(200) && colorValue > 1000)
                {
                    if (!colorLogic(colorValue)) // if we picked the wrong color we initialize the drop sequence
                    {
                        state = OuttakeState.INTAKE_DROP;
                        resetTimer();
                    } else
                    {
                        state = OuttakeState.TRANSFER_START;
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
                    if (delay(400))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    // this assumes that the sample is stuck in the chute so we spin the intake
                }
                break;
            case TRANSFER_START:
                if (delay(400) && intakeSubsystem.slideReached(slideTeleBase))
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                }
                if (delay(40))
                {
                    // this will hardstop the flap in the sample so the extendo can go back
                    intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    intakeClipHoldLogic(slideTeleBase, 8); // this controls the intake slides and the clip
                }
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(250))
                {
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                }
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                if (delay(40))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                }
                if (delay(120))
                {
                    intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                }
                break;
            case OUTTAKE_ADJUST: // this allows for the drivers to pre adjust heights
                if((gamepad1.right_bumper || gamepad2.right_bumper) && delay(100))
                {
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                }
                // this way the height control is independent of the of which type of deposit we are doing
                liftHeightLogic(gamepad2.a, gamepad2.x);
                outtakeTypeLogic(gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2,
                        gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2);
                break;
            case DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if (gamepad1.right_bumper)
                {
                    state = isBucket ? OuttakeState.SAMPLE_DEPOSIT : OuttakeState.SPECIMEN_DEPOSIT;
                    resetTimer();
                }
                outtakeTypeLogic(gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2,
                        gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2);
                liftHeightLogic(gamepad2.a, gamepad2.x);
                outtakeLiftPresets(isBucket, isLow); // this actually runs the lift
                armPositionLogic(isBucket);
            case SAMPLE_DEPOSIT: // this state is the automated sample deposit, no driver controls here
                if (delay(400))
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case SPECIMEN_DEPOSIT:
                if (delay(300))
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                outtakeSubsystem.ArmState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_SCORE);
                if (delay(40))
                {
                    outtakeLiftPresets(false, isLow, -40); // this actually runs the lift
                }
                break;
            case RETURN:
                if (outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    state = OuttakeState.READY;
                    resetTimer();
                }
                // couple things here to be done, fluctuate the intake
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
    public void intakeClipHoldLogic(int slideToPosition, int closeThreshold)
    {
        if (intakeSubsystem.slidePosition < closeThreshold)
        {
            if (globalTimer - intakeClipTimer > 90)
            {
                intakeSubsystem.intakeClipState(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideMotorRawControl(0);
            } else
            {
                intakeSubsystem.intakeClipState(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else
        {
            intakeSubsystem.intakeClipState(IntakeSubsystem.IntakeClipServoState.OPEN);
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            intakeClipTimer = globalTimer;
        }
    }
    public void outtakeLiftPresets(boolean isSample, boolean isLow)
    {
        if (isSample)
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);

        }
        else
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
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
    public void liftHeightLogic(boolean high, boolean low)
    {
        if (low)  isLow = true;
        else if (high) isLow = false;
    }
    public void armPositionLogic(boolean isBucket)
    {
        if (isBucket) outtakeSubsystem.ArmState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
        else outtakeSubsystem.ArmState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
    }
    public void outtakeTypeLogic(boolean isBucket, boolean isBar)
    {
        if (isBucket) this.isBucket = true;
        else if (isBar) this.isBucket= false;
    }

    public void readyOuttake()
    {
        intakeSubsystem.intakeFlapState(IntakeSubsystem.IntakeFlapServoState.DOWN);
        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
    }
}
