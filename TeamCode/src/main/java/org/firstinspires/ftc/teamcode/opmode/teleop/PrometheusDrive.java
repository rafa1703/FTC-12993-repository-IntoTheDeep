package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "PrometheusDrive", group = "A Drive")
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
    LoopTime loopTime = new LoopTime();
    enum OuttakeState {
        READY,
        INTAKE_EXTENDO,
        INTAKE_EXTENDO_SUB,
        INTAKE_EXTENDO_DROP,
        INTAKE,
        AFTER_EXTENDO,
        TRANSFER_START,
        TRANSFER_END,
        SPECIMEN_INTAKE,
        AFTER_SPECIMEN_INTAKE,
        OUTTAKE_ADJUST,
        HP_DEPOSIT,
        DEPOSIT,
        HANG_START,
        HANG_END,
        RETURN,
        MANUAL_ENCODER_RESET,
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDown intakeSlideSubBtn = new ToggleUpOrDown(1, 1, 0); // this instance will control
    double colorValue;
    int intakeSlideTarget;
    boolean dropped = false;
    boolean comingFromSub = false;
    boolean isLow = false, isSample = false;
    boolean manualControlLift = false;
    boolean cachedLiftPos = false;
    double tempLiftPos = 0;


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
            hardware = new GeneralHardware(hardwareMap, side);
            GlobalTimer = new ElapsedTime(System.nanoTime());
            sequenceTimer = globalTimer;
            intakeSubsystem = new IntakeSubsystem(hardware);
            outtakeSubsystem = new OuttakeSubsystem(hardware);
            driveBase = new DriveBaseSubsystem(hardware);
            prevIntakeFilterState = intakeSubsystem.intakeFilter;
            driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // hubs are inside hardware
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            globalTimer = GlobalTimer.milliseconds();

            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            intakeSubsystem.intakeReads(
                    state == OuttakeState.INTAKE ||
                    state == OuttakeState.INTAKE_EXTENDO ||
                    state == OuttakeState.INTAKE_EXTENDO_DROP ||
                    state == OuttakeState.INTAKE_EXTENDO_SUB
            );
            outtakeSubsystem.outtakeReads();
            outtakeSequence();


            telemetry.addData("State", state);
            telemetry.addData("Side", hardware.side);
            telemetry.addData("FilterState", intakeSubsystem.intakeFilter);
            telemetry.addData("Color logic", intakeSubsystem.colorLogic());
            loopTime.updateLoopTime(telemetry);
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
                        break;
                    } else if (gamepad1.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO;
                        intakeSlideTarget = slideTeleClose;
                        intakeSlideBtn.upToggle(gamepad1.left_bumper);
                        resetTimer();
                        break;
                    }
                    if (gamepad1LeftTrigger())
                    {
                        state = OuttakeState.SPECIMEN_INTAKE;
                        resetTimer();
                        break;
                    }
                    else if (gamepad2.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO_SUB;
                        intakeSlideTarget = slideTeleClose;
                        intakeSlideSubBtn.upToggle(gamepad2LeftTrigger());
                        resetTimer();
                        break;
                    }
                }
                intakeClipHoldLogic(slideTeleBase, 8);
                if (gamepad1.b || gamepad2.b)
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                break;
            case INTAKE_EXTENDO: // this state is just for using the extendo outside of the submersible
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(170))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER_FINISH);
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    if (gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if ((delay(700) && colorValue > 500) ||
                            (gamepad1RightTrigger() || gamepad2RightTrigger()))
                    {
                            state = OuttakeState.AFTER_EXTENDO;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            gamepad1.rumbleBlips(1);
                            resetTimer();
                            break;
                    }
                }
                break;
            case INTAKE_EXTENDO_SUB:
                intakeSlideBtn.upToggle(gamepad2.left_bumper);
                intakeSlideBtn.downToggle(gamepad2.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(40))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    if (intakeSubsystem.slideReached(intakeSlideTarget)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    if (gamepad1LeftTrigger() || gamepad2LeftTrigger())
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if ((delay(700) && colorValue > 500) ||
                            (gamepad1RightTrigger() || gamepad2RightTrigger()))
                    {
                        if (intakeSubsystem.colorLogic())
                        {
                            state = OuttakeState.AFTER_EXTENDO;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                            gamepad1.rumbleBlips(1);
                            resetTimer();
                            break;
                        }
                        else
                        {
                            state = OuttakeState.INTAKE_EXTENDO_DROP;
                            resetTimer();
                            break;
                        }
                    }
                }
                else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                break;
            case INTAKE_EXTENDO_DROP:
                if (colorValue < 800 && delay(2000))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    state = OuttakeState.INTAKE_EXTENDO;
                    resetTimer();
                    break;
                }
                colorValue = intakeSubsystem.getColorValue();
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
            case AFTER_EXTENDO: // this state only exist to introduce the behaviour that at transfer_start the slides are already in
                if (delay(300) && intakeSubsystem.isSlidesAtBase())
                {
                    state = OuttakeState.TRANSFER_START;
                    resetTimer();
                    break;
                }
                if (delay(90)) // gives time for the intake to go up
                    intakeClipHoldLogic(slideTeleBase, 5);

                break;
            case INTAKE:
                if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1.left_bumper);
                    resetTimer();
                    break;
                }
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                intakeClipHoldLogic(slideTeleBase, 5);
                colorValue = intakeSubsystem.getColorValue();
                if (delay(400) && colorValue > 500 ||
                        (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (gamepad2.right_bumper || gamepad1.right_bumper)))
                {
                    state = OuttakeState.TRANSFER_START;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    gamepad1.rumbleBlips(2);
                    resetTimer();
                    break;
                }
                break;
            case TRANSFER_START:
                if (delay(120) && outtakeSubsystem.liftAtBase() && intakeSubsystem.isSlidesAtBase())
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                    break;
                }

                intakeClipHoldLogic(slideTeleTransfer, 10); // this controls the intake slides and the clip
                outtakeSubsystem.liftMotorRawControl(-0.085);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    if (delay(90))
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);
                }
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(200))
                {
                    isSample = true;
                    isLow = false;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                    break;
                }
                if ((delay(40)))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(100))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    }
                    if (delay(140))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FINISH); // we might have to wait to go up
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER_FINISH);
                    }
                }
                break;
            case SPECIMEN_INTAKE:
                if (gamepad1.right_bumper || gamepad2.right_bumper)
                {
                    state = OuttakeState.AFTER_SPECIMEN_INTAKE;
                    resetTimer();
                    break;
                }
                if (delay(40))
                {
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntake);
                    if (delay(70))
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                    if (delay(180))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
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
                    isSample = false;
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntake + 2); // so like little bump up
                if (delay(130))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);
                }
                break;
            case OUTTAKE_ADJUST: // this allows for the drivers to pre adjust heights
                if ((gamepad1.right_bumper || gamepad2.right_bumper) && delay(300))
                {
                    state = OuttakeState.DEPOSIT;
                    resetTimer();
                    break;
                }
                if ((gamepad1.left_bumper || gamepad2.left_bumper) && delay(300))
                {
                    state = OuttakeState.HP_DEPOSIT;
                    resetTimer();
                    break;
                }
                // this way the height control is independent of the of which type of deposit we are doing
                liftHeightLogic(gamepad2.x, gamepad2.a);
                break;
            case HP_DEPOSIT:
                if (delay(200) && dropped)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                if (delay(40))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                    if (delay(120)) // arm goes over
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    }
                    if (delay(220)) outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.LOW);
                }

                break;
            case DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if ((gamepad1.right_bumper || gamepad2.right_bumper) && delay(400))
                {
                    state = isSample ? OuttakeState.SAMPLE_DROP : OuttakeState.SPECIMEN_DROP;
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);

                if (delay(90))
                {
                    if (!isSample)
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                    if (Math.abs(gamepad2.right_stick_y) > 0.05) // basically if d2 assumes the lift its on her
                        manualControlLift = true;

                    if (!manualControlLift)
                        outtakeLiftPresets(isSample, isLow); // this actually runs the lift
                    else outtakeSubsystem.liftMotorRawControl(-gamepad2.right_stick_y);

                    if (isSample)
                    {
                        if (outtakeLiftHasReachedPresets())
                        {// this reduces the huge backlash on the arm and improves liftPID
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        }
                        else if(!manualControlLift) // this is required or when d2 moves the lift the previous condition is false
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    }
                    else if (isLow)
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    }
                    else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                }
                break;
            case SAMPLE_DROP: // this state is the automated sample deposit, no driver controls here
                if (delay(200) && gamepad1.right_bumper)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case SPECIMEN_DROP: // this state is the automated specimen deposit, no driver controls here
                if (delay(800) && gamepad1.right_bumper)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                // this sequence should bump down the specimen
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                if (delay(40)) {
                    if (!manualControlLift)
                        outtakeLiftPresets(false, isLow, +7); // this actually runs the lift
                    else if (!cachedLiftPos)
                    {
                        tempLiftPos = outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition);
                        cachedLiftPos = true;
                    }
                    else outtakeSubsystem.liftToInternalPID(isLow ? tempLiftPos - 2 :  tempLiftPos + 7);
                }
                if (delay(800))
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                break;
            case RETURN:
                if ((outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos) && delay(650))
                        || delay(800))
                {
                    manualControlLift = false;
                    isSample = true;
                    cachedLiftPos = false;
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    state = OuttakeState.READY;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                if (delay(40))
                {
                    intakeSubsystem.intakeSpin(-0.6);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);

                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos - 5);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
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
                    break;
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
        if (    (gamepad1.b || gamepad2.b) &&
                (state != OuttakeState.READY) &&
                (state != OuttakeState.MANUAL_ENCODER_RESET) &&
                (state != OuttakeState.RETURN)
        )
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
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.YELLOW_ONLY; // toggles it off
            }
            if (gamepad2.dpad_right)
            {
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL; // both neutral and alliance
            }
            if (gamepad2.dpad_down)
            {
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY; // only alliance
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
    public void intakeClipHoldLogicWithoutPowerCutout(int slideToPosition, int closeThreshold){
        if (intakeSubsystem.slidePosition < closeThreshold) {
            if (globalTimer - intakeClipTimer > 100){
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(0,1);
            } else {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
                // intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
            }
        } else {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
            // intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
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
        if (isSample)
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
                case SIDE_ONLY: // just trying to give the drivers some feedback on the filter state
                    if(side == GeneralHardware.Side.Red) gamepad2.setLedColor(255, 0, 255, 2000);
                    else gamepad2.setLedColor(0, 100, 255, 2000);
                    break;
                case NEUTRAL:
                    gamepad2.setLedColor(255, 255, 255, 2000);
                    break;
                case YELLOW_ONLY:
                    gamepad2.setLedColor(255, 255, 0, 2000);
                    break;
                case OFF:
                    gamepad2.setLedColor(0, 0, 0, 2000);
                    break;
            }
            prevIntakeFilterState = newState;
        }
    }
    public boolean gamepad1RightTrigger()
    {
        return gamepad1.right_trigger > 0.2;
    }
    public boolean gamepad1LeftTrigger()
    {
        return gamepad1.left_trigger > 0.2;
    }
    public boolean gamepad2RightTrigger()
    {
        return gamepad2.right_trigger > 0.2;
    }
    public boolean gamepad2LeftTrigger()
    {
        return gamepad2.left_trigger > 0.2;
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
