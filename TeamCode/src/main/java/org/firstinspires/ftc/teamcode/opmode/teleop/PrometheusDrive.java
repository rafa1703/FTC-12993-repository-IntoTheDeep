package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "PrometheusDrive", group = "ADrive")
public class PrometheusDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    GeneralHardware.Side side = GeneralHardware.Side.Red;
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
        AFTER_SPECIMEN_INTAKE,
        OUTTAKE_ADJUST,
        HP_DEPOSIT,
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
    int outtakeSlidesTarget;
    boolean manualControlLift = false;
    boolean cachedLiftPos = false;
    double tempLiftPos = 0;
    boolean notTransferringFromExtendo = true;


    @Override
    public void runOpMode() throws InterruptedException
    {
        while (!isStarted()) { // initialization loop
            if (gamepad2.dpad_up || gamepad1.dpad_up) side = GeneralHardware.Side.Red;
            else if (gamepad2.dpad_down || gamepad1.dpad_down) side = GeneralHardware.Side.Blue;

            if (side == GeneralHardware.Side.Blue) gamepad2.setLedColor(0, 0, 255, 1000);
            else gamepad2.setLedColor(255, 0, 0, 1000);

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
            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // hubs are inside hardware
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            globalTimer = GlobalTimer.milliseconds();

            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            intakeSubsystem.intakeReads(state == OuttakeState.INTAKE || state == OuttakeState.INTAKE_DROP || state == OuttakeState.INTAKE_EXTENDO || state == OuttakeState.INTAKE_EXTENDO_DROP);
            outtakeSubsystem.outtakeReads();
            outtakeSequence();


            telemetry.addData("State", state);
            telemetry.addData("FilterState", intakeSubsystem.intakeFilter);
            telemetry.addData("Color value", intakeSubsystem.getColorValue());
            telemetry.addData("Color logic", intakeSubsystem.colorLogic());
            telemetry.update();
        }
    }
    // i will like properly tune delays for nats
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
                    } else if (gamepad1.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO;
                        intakeSlideTarget = slideTeleClose;
                        intakeSlideBtn.upToggle(gamepad1.left_bumper);
                        resetTimer();
                    }
                    if (gamepad2.left_bumper)
                    {
                        state = OuttakeState.SPECIMEN_INTAKE;
                        resetTimer();
                    }
                }
                intakeClipHoldLogic(slideTeleBase, 8);
                //intakeArmHeight();
                if (gamepad1.b || gamepad2.b)
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                break;
            case INTAKE_EXTENDO:
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;
                if (delay(200) && gamepad2.dpad_left)
                {
                    state = OuttakeState.INTAKE_EXTENDO_DROP;
                    resetTimer();
                    break;
                }
                intakeArmHeight();
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(170))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    if (gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (delay(1000) && colorValue > 500 ||
                            (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) ||
                            (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF &&
                                    (gamepad2.right_bumper || gamepad1.right_bumper)))
                    {
                        if (intakeSubsystem.colorLogic()
                                        && (gamepad2.right_bumper || gamepad1.right_bumper))
                        {
                            state = OuttakeState.TRANSFER_START;
                            notTransferringFromExtendo = false;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            gamepad1.rumbleBlips(2);
                            resetTimer();
                        } else if (!intakeSubsystem.colorLogic())// if we picked the wrong color we initialize the drop sequence
                        {
                            state = OuttakeState.INTAKE_EXTENDO_DROP;
                            resetTimer();
                        }
                    }
                }
                break;
            case INTAKE_EXTENDO_DROP:
                if ((colorValue < 800 && delay(2000)) || gamepad1.right_bumper || gamepad2.right_bumper)
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    state = OuttakeState.INTAKE_EXTENDO;
                    resetTimer();
                }
                colorValue = intakeSubsystem.getColorValue();
                intakeArmHeight();
                if (delay(400))
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget + 5);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);

                if (delay(50))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(600))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        // this assumes that the sample is stuck in the chute so we spin the intake
                    else
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drop
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
                if (delay(200) && colorValue > 500 ||
                        (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (gamepad2.right_bumper || gamepad1.right_bumper)))
                {
                    if (true) // if we picked the wrong color we initialize the drop sequence
                    {
                        state = OuttakeState.TRANSFER_START;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        gamepad1.rumbleBlips(2);
                        notTransferringFromExtendo = true;
                        resetTimer();
                    } else
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
                if (colorValue < 800 && delay(1000) && gamepad1.right_bumper)
                {
                    state = OuttakeState.INTAKE;
                    resetTimer();
                }
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                if (delay(200))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(400)) intakeSubsystem.intakeSlideInternalPID(10);

                    if (delay(600))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE); // this assumes that the sample is stuck in the chute so we spin the intake
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    //else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drops
                }
                break;
            case TRANSFER_START:
                if (notTransferringFromExtendo ? delay(600) : delay(950) && intakeSubsystem.isSlidesAtBase())
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
                if (intakeSubsystem.isSlidesAtBase())
                {
                    if (notTransferringFromExtendo ? delay(120) : delay(320))
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    if ((notTransferringFromExtendo ? delay(230) : delay(530)))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
                    }
                    if (notTransferringFromExtendo ? delay(400) : delay(400))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    }
                } else if (delay(35))
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                // naming makes no sense but this makes sure the arm i high when the slides come back
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(600))
                {
                    isBucket = true;
                    isLow = false;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                }
                intakeClipHoldLogic(slideTeleTransfer, 5); // this controls the intake slides and the clip
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos); // may be necessary an offset, hopefully not with box tube
                if ((delay(250) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos)) || delay(400))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(350))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    }
                    if (delay(400))
                    {// we wait a bit to to pivot
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    }
                    if (delay(440))
                    {
                        outtakeSubsystem.liftToInternalPID(5);
                        if (delay(500))
                        {
                            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER_FINISH);
                        }
                    }
                }
                break;
            case SPECIMEN_INTAKE:
                if (gamepad1.right_bumper || gamepad2.right_bumper)
                {
                    state = OuttakeState.AFTER_SPECIMEN_INTAKE;
                    resetTimer();
                }
                if (delay(40))
                {
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntake);
                    if (delay(70))
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                    if (delay(180))
                    {
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.INTAKE);
                    }
                    if (delay(230))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH); // this is so we can get closer to the wall
                    }
                }
                if (delay(250) && (gamepad2.left_bumper || gamepad1.left_bumper))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                }
                if (gamepad2.left_trigger > 0.4 || gamepad1.left_trigger > 0.4) // idk if d1 should have this tbh
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                }
                break;
            case AFTER_SPECIMEN_INTAKE: // AUTO state no driver controlers here
                if (delay(200))
                {
                    isBucket = false;
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntake + 2); // so like little bump up
                if (delay(130))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SPECIMEN);
                }
                break;
            case OUTTAKE_ADJUST: // this allows for the drivers to pre adjust heights
                if ((gamepad1.right_bumper || gamepad2.right_bumper) && delay(300))
                {
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                }
                if ((gamepad1.left_bumper || gamepad2.left_bumper) && delay(300))
                {
                    state = OuttakeState.HP_DEPOSIT;
                    resetTimer();
                }
                // this way the height control is independent of the of which type of deposit we are doing
                liftHeightLogic(gamepad2.x, gamepad2.a);
                break;
            case HP_DEPOSIT:
                if (delay(200) && (gamepad1.right_bumper || gamepad2.right_bumper))
                {
                    state = OuttakeState.SAMPLE_DROP;
                    resetTimer();
                    break;
                }
                if (delay(40))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                }
                if (delay(450))
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SAMPLE);
                else outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
                break;
            case DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if ((gamepad1.right_bumper || gamepad2.right_bumper) && delay(400))
                {
                    state = isBucket ? OuttakeState.SAMPLE_DROP : OuttakeState.SPECIMEN_DROP;
                    resetTimer();
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);

                if (delay(90))
                {
                    if (!isBucket)
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                    if (Math.abs(gamepad2.right_stick_y) > 0.05) // basically if d2 assumes the lift its on her
                        manualControlLift = true;

                    if (!manualControlLift)
                        outtakeLiftPresets(isBucket, isLow); // this actually runs the lift
                    else outtakeSubsystem.liftMotorRawControl(-gamepad2.right_stick_y);

                    if (isBucket)
                    {
                        if (outtakeLiftHasReachedPresets())
                        {// this reduces the huge backlash on the arm and improves liftPID
                            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SAMPLE);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        }
                        else if(!manualControlLift) // this is required or when d2 moves the lift the previous condition is false
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    }
                    else if (isLow)
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    }
                    else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                }
                break;
            case SAMPLE_DROP: // this state is the automated sample deposit, no driver controls here
                if (delay(200) && gamepad1.right_bumper)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case SPECIMEN_DROP: // this state is the automated specimen deposit, no driver controls here
                if (delay(800) && gamepad1.right_bumper)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                // this sequence should bump down the specimen
                outtakeSubsystem.armState(isLow ?
                        OuttakeSubsystem.OuttakeArmServoState.SPECIMEN : OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH, 0.1);
                if (delay(40)) {
                    if (!manualControlLift)
                        outtakeLiftPresets(false, isLow, -7); // this actually runs the lift
                    else if (!cachedLiftPos)
                    {
                        tempLiftPos = outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition);
                        cachedLiftPos = true;
                    }
                    else outtakeSubsystem.liftToInternalPID(isLow ? tempLiftPos - 2 :  tempLiftPos - 7);
                }
                if (delay(800))
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case RETURN:
                if ((outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos) && delay(650))
                        || delay(800))
                {
                    manualControlLift = false;
                    isBucket = true;
                    cachedLiftPos = false;
                    notTransferringFromExtendo = true;
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    state = OuttakeState.READY;
                    resetTimer();
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                if (delay(40))
                {
                    intakeSubsystem.intakeSpin(-0.6);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);

                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos - 5);
                    outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
                }
                if (delay(250))
                {
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                }
                if (delay(400))
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                else outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);

                if (delay(550))
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                else if (delay(40))
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                break;
            case MANUAL_ENCODER_RESET:
                if (delay(200) && gamepad2.right_bumper)
                {
                    outtakeSubsystem.liftMotorEncoderReset();
                    intakeSubsystem.intakeSlideMotorEncoderReset();
                    state = OuttakeState.RETURN;
                    resetTimer();
                }
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH); // holds the arm up
                if (delay(100))
                {
                    outtakeSubsystem.liftMotorRawControl(gamepad2.right_stick_y);
                    intakeSubsystem.intakeSlideMotorRawControl(gamepad2.right_trigger - gamepad2.left_trigger);
                }
                if (gamepad2.a) intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                if (gamepad2.x) intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                if (gamepad2.y) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                if (gamepad2.b) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                if (gamepad2.dpad_up) intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                if (gamepad2.dpad_down) intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                break;
        }
        // we run everything here that isn't state specific
        if ((gamepad1.b &&
                (state != OuttakeState.READY) &&
                (state != OuttakeState.MANUAL_ENCODER_RESET) &&
                (state != OuttakeState.RETURN)) ||
                (gamepad2.b &&
                (state != OuttakeState.READY) &&
                (state != OuttakeState.MANUAL_ENCODER_RESET) &&
                (state != OuttakeState.RETURN) &&
                (state != OuttakeState.INTAKE_EXTENDO) &&
                (state != OuttakeState.INTAKE_EXTENDO_DROP)
        ))
        {
            // can't reset if in manual reset lmao
            state = OuttakeState.RETURN; // if b is pressed at any state then return to ready
            resetTimer();
        }
        if (gamepad2.share)
        {
            state = OuttakeState.MANUAL_ENCODER_RESET;
            resetTimer();
        }
        if (state != OuttakeState.MANUAL_ENCODER_RESET)
        {
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
    }
    public void intakeArmHeight()
    {
        if (gamepad1.x || gamepad2.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DROP_HIGH);
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
            if (isLow) return outtakeSubsystem.liftReached((int) outtakeSubsystem.inchesToTicksSlidesMotor(OuttakeSubsystem.liftLowBucketPos));
            else return outtakeSubsystem.liftReached((int) outtakeSubsystem.inchesToTicksSlidesMotor(OuttakeSubsystem.liftHighBucketPos));

        }
        else
        {
            if (isLow) return outtakeSubsystem.liftReached((int) outtakeSubsystem.inchesToTicksSlidesMotor(OuttakeSubsystem.liftLowBarPos));
            else return outtakeSubsystem.liftReached((int) outtakeSubsystem.inchesToTicksSlidesMotor(OuttakeSubsystem.liftHighBarPos));
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
    /*public void outtakeTypeLogic(boolean isBucket, boolean isBar)
    {
        if (isBucket) this.isBucket = true;
        else if (isBar) this.isBucket= false;
    }*/

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
