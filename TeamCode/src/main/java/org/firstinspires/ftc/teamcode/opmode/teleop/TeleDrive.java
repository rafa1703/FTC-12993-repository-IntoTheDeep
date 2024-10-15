package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class TeleDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    GeneralHardware.Side side = GeneralHardware.Side.Red;
    // this matches the initial value in the intakeSubsystem
    IntakeSubsystem.IntakeFilter prevIntakeFilterState = IntakeSubsystem.IntakeFilter.NEUTRAL;
    enum OuttakeState {
        READY,
        INTAKE_EXTENDO,
        INTAKE_EXTENDO_DROP,
        INTAKE,
        INTAKE_DROP,
        TRANSFER_START,
        TRANSFER_END,
        SPECIMEN_INTAKE,
        OUTTAKE_ADJUST,
        DEPOSIT,
        SAMPLE_DEPOSIT,
        SPECIMEN_DEPOSIT,
        RETURN,
        MANUAL_ENCODER_RESET,
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    double colorValue;
    int intakeSlideTarget;
    boolean isLow = false, isBucket = false; // lol this is gonna work this way and i hate it

    @Override
    public void runOpMode() throws InterruptedException
    {
        while (!isStarted()) { // initialization loop
            if (gamepad2.dpad_up) side = GeneralHardware.Side.Red;
            else if (gamepad2.dpad_down) side = GeneralHardware.Side.Blue;

            if (side == GeneralHardware.Side.Blue) gamepad2.setLedColor(0, 0, 255, 1000);
            else gamepad2.setLedColor(255, 0, 0, 1000);

            telemetry.addData("Side", side);
            telemetry.update();
        }
        // idk if initializing it here will work
        hardware = new GeneralHardware(hardwareMap, side);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        sequenceTimer = globalTimer;
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        driveBase = new DriveBaseSubsystem(hardware);
        // hubs are inside hardware

        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == OuttakeState.INTAKE || state == OuttakeState.INTAKE_DROP || state == OuttakeState.INTAKE_EXTENDO);
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            outtakeSubsystem.outtakeReads();
            outtakeSequence();
            telemetry.addData("State", state);
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
                    if (gamepad1.right_bumper || gamepad2.right_bumper)
                    {
                        state = OuttakeState.INTAKE;
                        resetTimer();
                    }
                    else if (gamepad1.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        intakeSlideTarget = slideTeleClose;
                        intakeSlideBtn.upToggle(gamepad1.left_bumper);
                        resetTimer();
                    }
                    if (gamepad2.left_trigger > 0.2)
                    {
                        state = OuttakeState.SPECIMEN_INTAKE;
                        resetTimer();
                    }
                }
                intakeArmHeight();
                if (gamepad1.b || gamepad2.b) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
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

                    if (delay(200) && colorValue > 1000 ||
                            (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (gamepad2.right_bumper || gamepad1.right_bumper)))
                    {
                        if (intakeSubsystem.colorLogic())
                        {
                            state = OuttakeState.TRANSFER_START;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            gamepad1.rumbleBlips(2);
                            resetTimer();
                        }
                        else // if we picked the wrong color we initialize the drop sequence
                        {
                            state = OuttakeState.INTAKE_DROP;
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
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    state = OuttakeState.INTAKE_EXTENDO;
                    resetTimer();
                }
                if (delay(50))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    // this assumes that the sample is stuck in the chute so we spin the intake
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drop
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
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                // whenever we want to intake the intake goes down but with this the driver can hold the button to control it
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();
                if (delay(200) && colorValue > 1000 ||
                        (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (gamepad2.right_bumper || gamepad1.right_bumper)))
                {
                    if (intakeSubsystem.colorLogic()) // if we picked the wrong color we initialize the drop sequence
                    {
                        state = OuttakeState.TRANSFER_START;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        gamepad1.rumbleBlips(2);
                        resetTimer();
                    }
                    else
                    {
                        state = OuttakeState.INTAKE_DROP;
                        resetTimer();
                    }
                }
                break;
            case INTAKE_DROP:
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                if (colorValue < 1000 && delay(200))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    state = OuttakeState.INTAKE;
                    resetTimer();
                }
                if (delay(50))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE); // this assumes that the sample is stuck in the chute so we spin the intake
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drops
                }
                break;
            case TRANSFER_START:
                if (delay(200) && intakeSubsystem.slideReached(slideTeleBase))
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                }
                if (delay(40))
                {
                    // this will hardstop the flap in the sample so the extendo can go back
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    intakeClipHoldLogic(slideTeleTransfer, 8); // this controls the intake slides and the clip
                }
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(250) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                }
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos); // may be necessary an offset, hopefully not with box tube
                if (delay(40) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                }
                if (delay(80))
                {
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
                }
                if (delay(120))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                }
                break;
            case SPECIMEN_INTAKE:
                if (gamepad1.right_bumper || gamepad2.right_bumper)
                {
                    isBucket = false;
                    isLow = false;
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                }
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntake);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH); // this is so we can get closer to the wall
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.INTAKE);
                }
                if (delay(50) && (gamepad2.left_bumper || gamepad1.left_bumper))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                }
                if (gamepad2.left_trigger > 0.4 && gamepad1.left_trigger > 0.85) // idk if d1 should have this tbh
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                }
                liftHeightLogic(gamepad2.a, gamepad2.x);
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
            case SPECIMEN_DEPOSIT: // this state is the automated sample deposit, no driver controls here
                if (delay(300))
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                // this sequence should bump down the specimen
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_SCORE);
                if (delay(40)) outtakeLiftPresets(false, isLow, -40); // this actually runs the lift
                if (delay( 200)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case RETURN:
                if (outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    state = OuttakeState.READY;
                    resetTimer();
                }
                if (delay(0))
                {
                    intakeSubsystem.intakeSpin(-0.3);
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.READY);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);

                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
                }
                if (delay(100))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                break;
            case MANUAL_ENCODER_RESET:
                // just gonna code this if robot fucks up in drive practice
                break;

        }
        // we run everything here that isn't state specific
        if ((gamepad2.b || gamepad1.b) && (state != OuttakeState.READY) && (state != OuttakeState.MANUAL_ENCODER_RESET))
        {
            // can't reset if in manual reset lmao
            state = OuttakeState.RETURN; // if b is pressed at any state then return to ready
        }
        if (gamepad2.dpad_up)
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.OFF; // toggles it off
        }
        if (gamepad2.dpad_right)
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL; // both neutral and alliance
        }
        if (gamepad2.dpad_down)
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_SPECIFIC; // only alliance
        }
        updateGamepadLED(intakeSubsystem.intakeFilter); // this just make sure we are not queuing up infinite LED calls
    }
    public void intakeArmHeight()
    {
        if (gamepad1.x || gamepad2.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
        else if (gamepad1.a || gamepad2.a) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
    }

    public void intakeClipHoldLogic(int slideToPosition, int closeThreshold)
    {
        if (intakeSubsystem.slidePosition < closeThreshold)
        {
            if (globalTimer - intakeClipTimer > 90)
            {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideMotorRawControl(0);
            } else
            {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else
        {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
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
        if (isBucket) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
        else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
    }
    public void outtakeTypeLogic(boolean isBucket, boolean isBar)
    {
        if (isBucket) this.isBucket = true;
        else if (isBar) this.isBucket= false;
    }

    public void updateGamepadLED(IntakeSubsystem.IntakeFilter newState)
    {
        if (newState != prevIntakeFilterState)
        {
            switch (newState)
            {
                case SIDE_SPECIFIC: // just trying to give the drivers some feedback on the filter state
                    if(side == GeneralHardware.Side.Red) gamepad2.setLedColor(255, 0, 0, 180000);
                    else gamepad2.setLedColor(0, 0, 255, 180000);
                    break;
                case NEUTRAL:
                    gamepad2.setLedColor(255, 255, 0, 180000);
                    break;
                case OFF:
                    gamepad2.setLedColor(0, 0, 0, 180000);
                    break;
            }
            prevIntakeFilterState = newState;
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
