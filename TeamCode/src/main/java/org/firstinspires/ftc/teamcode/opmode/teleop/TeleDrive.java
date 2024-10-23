package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "PrometheusDrive", group = "Drive")
public class TeleDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    GeneralHardware.Side side = GeneralHardware.Side.Blue;
    // this matches the initial value in the intakeSubsystem
    IntakeSubsystem.IntakeFilter prevIntakeFilterState;
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
        SAMPLE_DROP,
        SPECIMEN_DROP,
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
            if (gamepad2.dpad_up || gamepad1.dpad_up) side = GeneralHardware.Side.Red;
            else if (gamepad2.dpad_down || gamepad1.dpad_down) side = GeneralHardware.Side.Blue;

            if (side == GeneralHardware.Side.Blue) gamepad1.setLedColor(0, 100, 255, 1000);
            else gamepad1.setLedColor(255, 0, 255, 1000);

            telemetry.addData("Side", side);
            telemetry.update();
        }
        if (opModeIsActive())
        {
            // idk if initializing it here will work
            hardware = new GeneralHardware(hardwareMap, side);
            GlobalTimer = new ElapsedTime(System.nanoTime());
            sequenceTimer = globalTimer;
            intakeSubsystem = new IntakeSubsystem(hardware);
            outtakeSubsystem = new OuttakeSubsystem(hardware);
            driveBase = new DriveBaseSubsystem(hardware);
            prevIntakeFilterState = intakeSubsystem.intakeFilter;
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
        }
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
            telemetry.addData("FilterState", intakeSubsystem.intakeFilter);
            telemetry.addData("Color value", intakeSubsystem.getColorValue());
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
                intakeClipHoldLogic(slideTeleBase, 8);
                //intakeArmHeight();
                if (gamepad1.b || gamepad2.b) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                break;
            case INTAKE_EXTENDO:
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();
                if (delay(140))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (gamepad1.share)
                    {
                        state = OuttakeState.INTAKE_EXTENDO_DROP;
                        resetTimer();
                    }
                    if (delay(200) && colorValue > 1000 || (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) ||
                            (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (gamepad2.right_bumper || gamepad1.right_bumper)))
                    {
                        if (intakeSubsystem.colorLogic() ||
                                ((intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF
                                        && (gamepad2.right_bumper || gamepad1.right_bumper))))
                        {
                            state = OuttakeState.TRANSFER_START;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            gamepad1.rumbleBlips(2);
                            resetTimer();
                        }
                        else // if we picked the wrong color we initialize the drop sequence
                        {
                            state = OuttakeState.INTAKE_EXTENDO_DROP;
                            resetTimer();
                        }
                    }
                }
                break;
            case INTAKE_EXTENDO_DROP:
                if (colorValue < 1000 && delay(1000) && gamepad1.right_bumper)
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    state = OuttakeState.INTAKE_EXTENDO;
                    resetTimer();
                }
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget + 5);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);

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
                intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                intakeClipHoldLogic(slideTeleBase, 5);
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
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        resetTimer();
                    }
                }
                break;
            case INTAKE_DROP:
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                if (colorValue < 200 && delay(1000) && gamepad1.right_bumper)
                {
                    state = OuttakeState.INTAKE;
                    resetTimer();
                }
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                if (delay(200))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400)) intakeSubsystem.intakeSlideInternalPID(10);

                    //if (delay(1500)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE); // this assumes that the sample is stuck in the chute so we spin the intake
                    //else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drops
                }
                break;
            case TRANSFER_START:
                if (delay(600) && intakeSubsystem.slideReached(slideTeleBase))
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                }
                if (delay(40))
                {
                    // this will hardstop the flap in the sample so the extendo can go back
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    intakeClipHoldLogic(slideTeleTransfer, 1); // this controls the intake slides and the clip
                }
                if (intakeSubsystem.slideReached(slideTeleBase) && delay(100))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    if (delay(150))
                    {
                        //intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    }
                    if (delay(230))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
                    }
                    if (delay(400))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    }
                }
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(530) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    isBucket = true;
                    isLow = false;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                }
                intakeClipHoldLogic(slideTeleTransfer, 5); // this controls the intake slides and the clip
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos); // may be necessary an offset, hopefully not with box tube
                if (delay(500) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(350)) intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    if (delay(360)) // we wait a bit to to pivot
                            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER_FINISH);
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
                if((gamepad1.right_bumper || gamepad2.right_bumper) && delay(300))
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
                if (gamepad1.right_bumper && delay(100000) && false)
                {
                    state = isBucket ? OuttakeState.SAMPLE_DROP : OuttakeState.SPECIMEN_DROP;
                    resetTimer();
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                outtakeTypeLogic(gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2,
                        gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2);
                liftHeightLogic(gamepad2.a, gamepad2.x);
                if (isBucket && delay(130) && !delay(200)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                if (delay(10000))
                {
                    outtakeLiftPresets(isBucket, isLow); // this actually runs the lift
                    if(outtakeLiftHasReachedPresets()) armPositionLogic(isBucket);
                    else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT); // no fancy cog math if arm is straight up
                }
                break;
            case SAMPLE_DROP: // this state is the automated sample deposit, no driver controls here
                if (delay(400) && false)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case SPECIMEN_DROP: // this state is the automated specimen deposit, no driver controls here
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
                if (outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos) && delay(500))
                {
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    state = OuttakeState.READY;
                    resetTimer();
                }
                if (delay(40))
                {
                    intakeSubsystem.intakeSpin(-0.3);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);

                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                }
                if (delay(400))
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                else outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (delay(200))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                break;
            case MANUAL_ENCODER_RESET:
                // just gonna code this if robot fucks up in drive practice
                break;
                // driven 77, driver 14, 5.5
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
    public boolean outtakeLiftHasReachedPresets()
    {
        if (isBucket)
        {
            if (isLow) return outtakeSubsystem.liftReached(OuttakeSubsystem.liftLowBucketPos);
            else return outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos);

        }
        else
        {
            if (isLow) return outtakeSubsystem.liftReached(OuttakeSubsystem.liftLowBarPos);
            else return outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBarPos);
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
                    if(side == GeneralHardware.Side.Red) gamepad2.setLedColor(255, 0, 255, 2000);
                    else gamepad2.setLedColor(0, 100, 255, 2000);
                    break;
                case NEUTRAL:
                    gamepad2.setLedColor(255, 255, 0, 2000);
                    break;
                case OFF:
                    gamepad2.setLedColor(0, 0, 0, 2000);
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
