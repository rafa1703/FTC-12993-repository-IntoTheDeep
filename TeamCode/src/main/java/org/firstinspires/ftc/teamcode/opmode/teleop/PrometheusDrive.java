package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleFallingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownWithLimit;
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
        SAMPLE_DEPOSIT,
        SPECIMEN_DEPOSIT,
        HANG_START,
        HANG_END,
        RETURN,
        MANUAL_ENCODER_RESET,
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDownWithLimit intakeSlideSubBtn = new ToggleUpOrDownWithLimit(1, 1, 0, 4); // this instance will control
    ToggleRisingEdge toggleRisingEdge = new ToggleRisingEdge();
    ToggleFallingEdge toggleDropFallingEdge = new ToggleFallingEdge();
    int intakeSLideIncrement = 5; // in
    double colorValue;
    double intakeSlideTarget;
    boolean dropped = false;
    boolean isSampleLow = false, isSpecimenLow = false; // this start at high and remembers
    boolean isSample = false;
    boolean goToIntake = false;


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
                    state == OuttakeState.INTAKE_EXTENDO_SUB ||
                    state == OuttakeState.SPECIMEN_DEPOSIT
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
        driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
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
                    if (gamepad1LeftTrigger() && gamepad1.dpad_up)
                    {
                        state = OuttakeState.HANG_START;
                        resetTimer();
                        break;
                    }
                    if (gamepad2LeftTrigger())
                    {
                        state = OuttakeState.SPECIMEN_INTAKE;
                        resetTimer();
                        break;
                    }
                    else if (gamepad2.left_bumper)
                    {
                        state = OuttakeState.INTAKE_EXTENDO_SUB;
                        intakeSlideTarget = intakeSLideIncrement;
                        intakeSlideSubBtn.upToggle(gamepad2.left_bumper);
                        toggleRisingEdge.mode(gamepad2.left_bumper);
                        resetTimer();
                        break;
                    }
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
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
                // changes done here also have to be done in spec_deposit
                intakeSlideSubBtn.upToggle(gamepad2.left_bumper);
                intakeSlideSubBtn.downToggle(gamepad2.right_bumper, 1);
                if (toggleRisingEdge.mode(gamepad2.left_bumper || gamepad2.right_bumper)) // if we use bumpers we can go to predetermined positions
                {
                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
                }
                else
                {
                    intakeSlideTarget += -gamepad2.left_stick_y * 0.8; // 0.8 in (20 in per second)
                    if (intakeSlideTarget > slideTeleFar) intakeSlideTarget = slideTeleFar; // caps for extension limit
                }

                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(40))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    intakeArmHeight();
                    if (gamepad2LeftTrigger())
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
                if (colorValue < 600 && delay(2000))
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
                    if (delay(500))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else if (delay(350)) // this assumes that the sample is stuck in the chute so we spin the intake
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
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
                if (delay(450) && outtakeSubsystem.liftAtBase() && intakeSubsystem.isSlidesAtBase())
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                    break;
                }

                intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                outtakeSubsystem.liftMotorRawControl(-0.085);
                if (delay(90))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER);
                    if (delay(240))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    }
                    if (delay(360))
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                }
                break;
            case TRANSFER_END:
                // so this is when the claw will grip and we are assuming that the slides are at transfer position
                if (delay(410))
                {
                    // we want to remember if we went low
                    isSample = true;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
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
                    if (delay(200))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FINISH); // we might have to wait to go up
                        outtakeSubsystem.liftMotorRawControl(0);
                    }
                    if (delay(300))
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER_FINISH);
                }
                break;
            case SPECIMEN_INTAKE:
                if (gamepad1.right_bumper || gamepad2.right_bumper)
                {
                    state = OuttakeState.AFTER_SPECIMEN_INTAKE;
                    resetTimer();
                    break;
                }

                if (delay(500))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE); // this makes the arm go over
                    if (delay(700)) // tune this so it only runs when the arm has reached intake pos
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    }
                }
                else
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.PERPENDICULAR);
                    if (delay(200))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                    }
                }
                if (gamepad1LeftTrigger() || gamepad2LeftTrigger()) // this just exists in case the claw comes in closed
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                }
                break;
            case AFTER_SPECIMEN_INTAKE: // AUTO state no driver controls here
                if (delay(400))
                {
                    isSample = false;
                    state = OuttakeState.SPECIMEN_DEPOSIT;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                if (delay(150))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                    if (delay(300))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    }
                }
                break;
            case OUTTAKE_ADJUST: // this allows for the drivers to pre adjust heights
                if ((gamepad1.right_bumper || gamepad2.right_bumper) && delay(300))
                {
                    state = isSample ? OuttakeState.SAMPLE_DEPOSIT : OuttakeState.SPECIMEN_DEPOSIT;
                    intakeSlideBtn.OffsetTargetPosition = 0; // this makes sure the extendo doesn't go out when we go into the deposit state
                    resetTimer();
                    break;
                }
                if ((gamepad1.left_bumper || gamepad2.left_bumper) && delay(300))
                {
                    state = OuttakeState.HP_DEPOSIT;
                    resetTimer();
                    break;
                }
                if (gamepad2LeftTrigger())
                {
                    goToIntake = true; // this allows d2 to pre going to intake after
                }
                // this way the height control is independent of the of which type of deposit we are doing
                liftHeightLogic(gamepad2.x, gamepad2.a);
                break;
            case HP_DEPOSIT:
                if (delay(200) && dropped)
                {
                    state = goToIntake ? OuttakeState.SPECIMEN_INTAKE : OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                if (gamepad2LeftTrigger())
                {
                    goToIntake = true; // this allows d2 to pre going to intake after
                }
                if (!dropped)
                {
                    if (delay(60)) // rail has time to go up
                    {
                        if (delay(120)) // arm goes over
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        }
                    }
                    if (delay(220))
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.LOW);
                    else
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                }
                if (delay(400) && (gamepad1.right_bumper || gamepad2.right_bumper))
                {
                    dropped = true;
                    resetTimer();
                }
                break;
            case SAMPLE_DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if (toggleDropFallingEdge.mode(gamepad1.right_bumper) && delay(400) && dropped)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);
                outtakeLiftPresets(); // this just runs the correct height for the lift
                if (!dropped)
                {
                    if (delay(40))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SAMPLE);
                        if (delay(140))
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                    }
                    if (delay(140) && gamepad1.right_bumper)
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        toggleDropFallingEdge.mode(gamepad1.right_bumper);
                        dropped = true;
                        resetTimer();
                    }
                }
                break;
            case SPECIMEN_DEPOSIT: // we want to deposit and intake
                if (delay(200) && gamepad1.right_bumper && dropped)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);
                if (delay(40))
                {
                    outtakeLiftPresets(); // this just runs the correct height for the lift
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH); // so we clear the submersible
                }
                if (!dropped)
                {
                    if (delay(40))
                    {
                        outtakeArmAndRailSpecimenPresets();
                        if (gamepad1.right_bumper)
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                            dropped = true;
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                }

                // the intake part
                intakeSlideSubBtn.upToggle(gamepad2.left_bumper);
                intakeSlideSubBtn.downToggle(gamepad2.right_bumper, 1);
                if (toggleRisingEdge.mode(gamepad2.left_bumper || gamepad2.right_bumper))
                {
                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
                }
                else
                {
                    intakeSlideTarget += -gamepad2.left_stick_y * 0.8; // 0.8 in (20 in per second)
                    if (intakeSlideTarget > slideTeleFar) intakeSlideTarget = slideTeleFar; // caps for extension limit
                }
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(80))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    intakeArmHeight();
                    if (gamepad1LeftTrigger() || gamepad2LeftTrigger())
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else if (dropped) // this makes so the brushes only run after we have dropped
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (
                            (delay(300) && colorValue > 500) ||
                            (gamepad1RightTrigger() || gamepad2RightTrigger()) &&
                                    dropped)
                    {
                        if (intakeSubsystem.colorLogic())
                        {
                            state = OuttakeState.AFTER_EXTENDO;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH); // this makes sure arm is up
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
                break;
            case HANG_START:
                if (gamepad1.right_bumper && delay(40))
                {
                    state = OuttakeState.HANG_END;
                    resetTimer();
                    break;
                }
                driveBase.HangState(DriveBaseSubsystem.HangState.OUT);
                break;
            case HANG_END:
                driveBase.HangState(DriveBaseSubsystem.HangState.UP);
                // we might have to make the intake slides go out
                break;
            case RETURN:
                if (outtakeSubsystem.liftAtBase() && delay(450))
                {
                    isSample = true;
                    goToIntake = false;
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    state = OuttakeState.READY;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(slideTeleBase, 10);
                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                if (delay(40))
                {
                    driveBase.HangState(DriveBaseSubsystem.HangState.READY); // this will effectively only run once
                    if (outtakeSubsystem.isArmOver())
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);
                    }
                    else
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);
                    if (delay(100))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
                    }
                    if (delay(190))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                        intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    }
                }
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
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY); // holds the arm up
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                if (delay(100))
                {
                    outtakeSubsystem.liftMotorRawControl(-gamepad2.right_stick_y);
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
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.YELLOW_ONLY; // yellow only
            }
            if (gamepad2.dpad_right)
            {
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL; // both neutral and alliance
            }
            if (gamepad2.dpad_down)
            {
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY; // only alliance
            }
            if (gamepad2.dpad_left)
            {
                intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.OFF; // toggles it off
            }
            updateGamepadLED(intakeSubsystem.intakeFilter); // this just make sure we are not queuing up infinite LED calls
        }
    }
    public void intakeArmHeight()
    {
        if (gamepad1.x || gamepad2.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
        else if (gamepad1.a || gamepad2.a) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
    }

    public void intakeClipHoldLogic(double slideToPosition, int closeThreshold)
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
    public void intakeClipHoldLogicWithoutPowerCutout(double slideToPosition, int closeThreshold){
        if (intakeSubsystem.slidePosition < closeThreshold) {
            if (globalTimer - intakeClipTimer > 100){
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(0);
            } else {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            intakeClipTimer = globalTimer;
        }
    }
    public void outtakeLiftPresets()
    {
        if (isSample)
        {
            if (isSampleLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
        }
        else
        {
            if (isSpecimenLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
        }
    }
    public void outtakeArmAndRailSpecimenPresets() // am gonna assume we only need a single arm position
    {
        if (isSpecimenLow)
        {
            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_LOW);
        }
        else
        {
            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
        }
        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
    }
    public void liftHeightLogic(boolean high, boolean low)
    {
        if (isSample)
        {
            if (low) isSampleLow = true;
            else if (high) isSampleLow = false;
        }
        else
        {
            if (low) isSpecimenLow = true;
            else if (high) isSpecimenLow = false;
        }
    }

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
