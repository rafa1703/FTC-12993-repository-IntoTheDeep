package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.system.accessory.SideAfterAuto;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownCircular;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownWithLimit;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "Sentinel Drive", group = "A")
public class SentinelDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer, turretTimer, internalTimer;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    GeneralHardware.Side side = SideAfterAuto.side;
    // this matches the initial value in the intakeSubsystem
    IntakeSubsystem.IntakeFilter prevIntakeFilterState;
    LoopTime loopTime = new LoopTime();

    enum OuttakeState {
        READY,
        INTAKE_EXTENDO,
        INTAKE_EXTENDO_SPEC,
        INTAKE_EXTENDO_SUB,
        INTAKE,
        AFTER_EXTENDO,
        TRANSFER_START,
        TRANSFER_END,
        SPECIMEN_INTAKE,
        OUTTAKE_ADJUST,
        HP_DEPOSIT,
        HP_DEPOSIT_EXTENDO,
        SAMPLE_DEPOSIT,
        SPECIMEN_DEPOSIT,
        SPECIMEN_DEPOSIT_BACK,
        SPECIMEN_DEPOSIT_BACK2,
        HANG_START,
        HANG_LEVEL2,
        HANG_LEVEL3,
        HANG_END,
        RETURN,
        MANUAL_ENCODER_RESET,
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDownWithLimit intakeSlideSubBtn = new ToggleUpOrDownWithLimit(1, 1, 0, 4);
    ToggleUpOrDown liftFineAdjustBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDown liftFineAdjustIntakeBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDownCircular togglePivot = new ToggleUpOrDownCircular(1, 1, 2, 7);
    ToggleUpOrDownWithLimit intakeArmToggle = new ToggleUpOrDownWithLimit(1, 1,0, 2);
    ToggleUpOrDownWithLimit intakeTurretToggle = new ToggleUpOrDownWithLimit(1, 1, 0, 4);
    ToggleRisingEdge toggleRisingEdge = new ToggleRisingEdge();
    ToggleRisingEdge toggleRisingEdgeD2Intake = new ToggleRisingEdge();
    ToggleRisingEdge secondToggleForTheDrop = new ToggleRisingEdge();
    ToggleRisingEdge toggleFineAdjustLift = new ToggleRisingEdge();
    ToggleRisingEdge toggleIntakeEdgeCase = new ToggleRisingEdge();
    ToggleRisingEdge toggleAutoSampleOuttake = new ToggleRisingEdge();
    ToggleRisingEdge toggleExtendoReturningFormBars = new ToggleRisingEdge();
    ToggleRisingEdge intakeTurretModeToggle = new ToggleRisingEdge();
    ToggleRisingEdge bButtonToggle = new ToggleRisingEdge();
    ToggleRisingEdge autoAlignToggle = new ToggleRisingEdge();
    ToggleRisingEdge hpExtendoToggle = new ToggleRisingEdge();
    ToggleRisingEdge toggleAuraTransfer  = new ToggleRisingEdge();
    ToggleRisingEdge toggleMetaTransfer  = new ToggleRisingEdge();
    IntakeSubsystem.IntakeTurretServoState turretServoState = IntakeSubsystem.IntakeTurretServoState.STRAIGHT;
    int intakeSLideIncrement = 5; // in
    double colourValue;
    double sampleThreshold = 850;
    double intakeSlideTarget;
    double liftTarget;
    double turretLastAngle;
    boolean intakeEdgeCase = false;
    boolean dropped = false;
    boolean isSampleLow = false, isSpecimenLow = false; // this start at high and remembers
    boolean wasSpecLow = false;
    boolean isSample = false;
    boolean specOnLeft = true;
    boolean goToIntake = false;
    boolean goToSampleDeposit = false;
    boolean goToHPDeposit = false;
    boolean goToHPExtendoDeposit;
    boolean fineAdjustingLiftIntake = false;
    boolean intakeTurretUsingPresets = true;
    boolean fineAdjustingIntakeSlides;
    boolean fineAdjustingLiftSpec = false;
    int liftFineAdjustSpecLowCache;
    int liftFineAdjustSpecHighCache;
    double liftFineAdjustIntakeCache;
    boolean autoSampleOuttake = true; // this should start as true ig
    boolean intaked;
    boolean intakingFromBars;
    boolean autoAlignSpecDeposit = true;
    boolean autoAlignSampleDeposit = true; // this starts off
    boolean autoAlignHpDeposit = true;
    boolean autoAlignSpecIntake = false;
    boolean autoAlignSpecDepositBack = true;
    boolean fineAdjustingLiftSample;
    boolean armWentDown;
    boolean extendoReturnFromBars = false; // default is false as we should most intake from sample sides, this should also automatically be true in spec dep
    boolean auraTransferEdgeCase;
    boolean metaTransferEdgeCase;
    boolean extended;
    boolean turretAngleCached;

    IMU imu;
    double angularVel, heading;


    @Override
    public void runOpMode() throws InterruptedException
    {
        while (!isStarted()) { // initialization loop
//            if (gamepad2.dpad_up || gamepad1.dpad_up) side = GeneralHardware.Side.Red;
//            else if (gamepad2.dpad_down || gamepad1.dpad_down) side = GeneralHardware.Side.Blue;
//
//            if (side == GeneralHardware.Side.Blue) gamepad2.setLedColor(0, 0, 255, 1000);
//            else gamepad2.setLedColor(255, 0, 0, 1000);
            telemetry.addData("Side", side);
            telemetry.update();
        }
        if (opModeIsActive())
        {
            hardware = new GeneralHardware(hardwareMap, side);
            GlobalTimer = new ElapsedTime(System.nanoTime());
            sequenceTimer = globalTimer;
            intakeSubsystem = new IntakeSubsystem(hardware);
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.OFF;
            outtakeSubsystem = new OuttakeSubsystem(hardware);
            driveBase = new DriveBaseSubsystem(hardware);
            prevIntakeFilterState = intakeSubsystem.intakeFilter;
            driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            imu = hardware.imu;
            //imu.initialize()
            imu.resetYaw();


            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
            outtakeSubsystem.lockServoState(OuttakeSubsystem.OuttakeLockServoState.OPEN);
//            outtakeSubsystem.turretSpinTo(180);

            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
            outtakeSubsystem.cacheTurretInitialPosition();
        }
        // hubs are inside hardwaren
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            globalTimer = GlobalTimer.milliseconds();

            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            intakeSubsystem.intakeReads(
                    state == OuttakeState.INTAKE ||
                    state == OuttakeState.INTAKE_EXTENDO ||
                    state == OuttakeState.INTAKE_EXTENDO_SPEC ||
                    state == OuttakeState.HP_DEPOSIT_EXTENDO ||
                    state == OuttakeState.INTAKE_EXTENDO_SUB ||
                    state == OuttakeState.SPECIMEN_DEPOSIT
            );
            outtakeSubsystem.outtakeReads(
                    true
//                    state == OuttakeState.AFTER_EXTENDO ||
//                        state == OuttakeState.TRANSFER_START ||
//                        state == OuttakeState.TRANSFER_END ||
//                        state == OuttakeState.SPECIMEN_DEPOSIT ||
//                        state == OuttakeState.SPECIMEN_INTAKE ||
//                        state == OuttakeState.SAMPLE_DEPOSIT
            );
            updateHeading();

            outtakeSequence();


            telemetry.addData("State", state);
            telemetry.addData("Side", hardware.side);
            telemetry.addData("Heading", heading);
            telemetry.addData("Heading reduced", Angles.reduceDegrees(heading));
            telemetry.addData("slides position", intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition));
            telemetry.addLine();
            telemetry.addData("LiftM amps", hardware.outtakeLiftM.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Turret amps", hardware.turretM.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("IntakeSlides amps", hardware.intakeSlidesM.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine();
            telemetry.addData("SpecOnLeft", specOnLeft);
            telemetry.addData("Lift pos", outtakeSubsystem.liftPosition);
            telemetry.addData("Is lift at base", outtakeSubsystem.liftAtBase());
            telemetry.addData("Is slides at base", intakeSubsystem.isSlidesAtBase());
            telemetry.addData("Turret position", outtakeSubsystem.turretIncrementalPosition);
            telemetry.addData("Turret initial offset", outtakeSubsystem.initialOffsetPosition);
            telemetry.addData("IntakeSLideBtn offset pos", intakeSlideBtn.OffsetTargetPosition);
            telemetry.addData("IntakeSLide target", intakeSlideTarget);
            telemetry.addData("Aura transfer", auraTransferEdgeCase);
            telemetry.addData("Meta transfer", metaTransferEdgeCase);
//            telemetry.addData("Auto align", autoAlign);
            telemetry.addData("Go hp extendo", goToHPExtendoDeposit);
//            telemetry.addData("Side after auto", SideAfterAuto.side);
//            telemetry.addData("Go to intake", goToIntake);
//            telemetry.addData("Go to Deposit", goToDeposit);
//            telemetry.addData("Go to HpDeposit", goToHPDeposit);
//            telemetry.addData("IsArmOver", outtakeSubsystem.isArmOver());
            telemetry.addData("Intake edge case", intakeEdgeCase);
            telemetry.addData("Turret angle", outtakeSubsystem.turretAngle);
            telemetry.addData("Turret angle wrapped", Angles.normalizeDegrees(outtakeSubsystem.turretAngle));
            telemetry.addData("Outtake Arm pos", outtakeSubsystem.getArmPos());
            telemetry.addData("Angular velocity", angularVel);

            telemetry.addData("FilterState", intakeSubsystem.intakeFilter);
            telemetry.addData("Color value", intakeSubsystem.getColorValue());
//            telemetry.addData("OuttakePos", outtakeSubsystem.liftPosition);
//            telemetry.addData("IntakePos", intakeSubsystem.slidePosition);
//            telemetry.addData("Dropped", dropped);
//            telemetry.addData("ClimbPos", driveBase.getClimbPosition());
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
        }
        if (hardware != null)
            driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // i will like properly tune delays for nats
    public void outtakeSequence()
    {
        switch (state)
        {
            case READY:
                if (gamepad1.right_bumper)
                {
                    state = OuttakeState.INTAKE;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    intakeTurretToggle.OffsetTargetPosition = 2;
                    resetTimer();
                    break;
                } else if (gamepad1RightTrigger())
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1RightTrigger());
                    intakeSlideBtn.OffsetTargetPosition = 1;
                    intakeTurretToggle.OffsetTargetPosition = 2;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                } else if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO_SUB;
                    //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    intakeSlideTarget = 23.2;
                    intakeSlideSubBtn.upToggle(gamepad1.right_bumper);
                    intakeSlideSubBtn.downToggle(gamepad1.left_bumper,1);
                    intakeSlideSubBtn.OffsetTargetPosition = 3;
                    intakeArmToggle.OffsetTargetPosition = 0;
                    intakeTurretToggle.upToggle(gamepad2RightTrigger());
                    intakeTurretToggle.downToggle(gamepad2LeftTrigger(),1);
                    intakeTurretToggle.OffsetTargetPosition = 2;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    internalTimerReset();
                    resetTimer();
                    break;
                }
                else if (gamepad1LeftTrigger())
                {
                    state = OuttakeState.INTAKE_EXTENDO_SPEC;
                    intakeSlideTarget = slideTeleFar;
                    intakeSlideBtn.upToggle(gamepad1LeftTrigger());
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                }

                if (gamepad1.share && false)
                {
                    state = OuttakeState.HANG_LEVEL3; // i need to improve this sequencing
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                if (gamepad2LeftTrigger())
                {
                    state = OuttakeState.SPECIMEN_INTAKE;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }

                if (gamepad1.b || gamepad2.b) // might have the slides go out for a bit to get stuff unstuck maybe the arm goes forward too?
                {
                    if (bButtonToggle.mode(gamepad1.b || gamepad2.b))
                    {
                        internalTimerReset();
                    }
                    if (internalDelay(400))
                    {
                        intakeSubsystem.intakeSpin(-0.4);
                    }
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE); // this might unstransfer, so samples don't get stuck
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT);
                }
                else
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
                }

                intakeClipHoldLogic(0, 8);
                if (!gamepad1.dpad_down)
                {
                    outtakeKeepTurretBack();
                }
                else outtakeSubsystem.turretRawControl(0);
                hpExtendoToggle();
                autoSampleOuttakeToggle(gamepad1.a);
                break;
            case INTAKE_EXTENDO: // this state is just for using the extendo outside of the submersible
                intakeSlideBtn.upToggle(gamepad1RightTrigger());
                intakeSlideBtn.downToggle(gamepad1LeftTrigger(), 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;


                if (delay(50))
                {
                    colourValue = intakeSubsystem.getColorValue();
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);

                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                    hpExtendoToggle();
                    autoSampleOuttakeToggle(gamepad1.a);
                    outtakeKeepTurretBack();
                    if (delay(120)) // waits until the arm is out
                    {
                       intakeTurretPresetsOrNot();
                    }

                    if ((delay(350) && colourValue >= sampleThreshold) ||
                        gamepad1.left_stick_button)
                    {
                            state = OuttakeState.AFTER_EXTENDO;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            intakeTurretUsingPresets = false;
                            isSample = true;
                            gamepad1.rumbleBlips(1);
                            resetTimer();
                            break;
                    }
                }
                break;
            case INTAKE_EXTENDO_SPEC:
                intakeSlideBtn.upToggle(gamepad1LeftTrigger());
                intakeSlideBtn.downToggle(gamepad1RightTrigger(), 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleFar;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleClose;


                if (delay(50))
                {
                    colourValue = intakeSubsystem.getColorValue();
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);

                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

//                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                    outtakeKeepTurretBack();

                    // sets up for front transfer
//                    outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
//                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT);
//                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);

                    specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.dpad_right);
                    if (delay(120)) // waits until the arm is out
                    {
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    }

                    if ((delay(700) && colourValue >= sampleThreshold) ||
                            gamepad1.left_stick_button)
                    {
                        state = OuttakeState.AFTER_EXTENDO;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        intakeTurretUsingPresets = false;
                        isSample = false;
                        gamepad1.rumbleBlips(1);
                        resetTimer();
                        break;
                    }
                }
                break;
            case INTAKE_EXTENDO_SUB: // this is the worst ftc game ever created
                // the intake part
                intakeSlideSubBtn.upToggle(gamepad1.right_bumper);
                intakeSlideSubBtn.downToggle(gamepad1.left_bumper, 1);
                if (intakeSlideSubBtn.OffsetTargetPosition == 1) intakeSlideTarget = 7;
                if (intakeSlideSubBtn.OffsetTargetPosition == 2) intakeSlideTarget = 18;
                if (intakeSlideSubBtn.OffsetTargetPosition == 3) intakeSlideTarget = 23.2;

//                intakeSlideSubBtn.upToggle(gamepad2.right_bumper);
//                intakeSlideSubBtn.downToggle(gamepad2.left_bumper, 1);
//
//                if (toggleRisingEdgeD2Intake.mode(gamepad2.left_bumper || gamepad2.right_bumper))
//                {
//                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
//                } else
    //                {
    //                intakeSlideTarget += -gamepad2.left_stick_y * 0.95; // 0.8 in (20 in per second)
    //                if (intakeSlideTarget > slideExtensionLimit)
    //                    intakeSlideTarget = slideExtensionLimit; // caps for extension limit
    //                if (intakeSlideTarget < -2)
    //                {
    //                    intakeSlideTarget = -2; // caps for extension limit
    //                }
//                }
                colourValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);


                if (delay(110))
                {
                    if (Math.abs(gamepad2.left_stick_y) > 0)
                    {
                        fineAdjustingIntakeSlides = true;
                    }
                    else if (gamepad1.left_bumper || gamepad1.right_bumper)
                    {
                        fineAdjustingIntakeSlides = false;
                    }

                    if (!fineAdjustingIntakeSlides)
                        intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    else intakeSubsystem.intakeSlideMotorRawControl(-gamepad2.left_stick_y * 0.8);

                    outtakeKeepTurretBack();
                    hpExtendoToggle();
                    autoSampleOuttakeToggle(gamepad1.a);
                    extendoReturnFromBarsToggle(gamepad1RightTrigger() || gamepad2.dpad_up);
                    auraTransferToggle(gamepad2.dpad_right);
                    metaTransferToggle(gamepad2.dpad_left);

                    if (toggleIntakeEdgeCase.mode(gamepad2.dpad_down || gamepad1.dpad_down))
                    {
                        intakeEdgeCase = !intakeEdgeCase;
                        if (!intakeEdgeCase)
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND); // makes arm go up before turret turns
                        }
                        internalTimerReset();
                    }

                    if (!intakeEdgeCase)
                    {
                        if (armWentDown)
                        {
                            if (internalDelay(300)) intakeSubArmHeight(); // if we want to exit the edge case
                        }
                        else if (intakeSubsystem.slideOverPosition(intakeSlideTarget * 0.85) || delay(600))
                        {
                            intakeArmToggle.OffsetTargetPosition = 2;
                            armWentDown = true;
                        }
                        if (internalDelay(250)) intakeTurretPresetsOrNot(); // if we want to exit the edge case
                    }
                    else
                    {
                        if (internalDelay(300))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                        }
                        else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.AROUND);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }

                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else if (intakeArmToggle.OffsetTargetPosition > 1)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (
                            (delay(300) && (colourValue >= sampleThreshold)
                                    || gamepad1.left_stick_button)
                    )
                    {
                        state = OuttakeState.AFTER_EXTENDO;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        intakeSubsystem.intakeArm(extendoReturnFromBars ? IntakeSubsystem.IntakeArmServoState.HORIZONTAL :
                                IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK); // this makes sure arm is up
                        gamepad1.rumbleBlips(2);
                        isSample = true;
                        resetTimer();
                        break;
                    }
                }
                else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                break;
            case AFTER_EXTENDO: // this might be necessary to check the colour and go back the slides
                if ((delay(200) && intakeSubsystem.isSlidesAtBase()) || (delay(900)))
                { // checks hpDepExtendo -> edgeCases -> intakingFromBars
                    state = goToHPExtendoDeposit ? OuttakeState.HP_DEPOSIT_EXTENDO :
                            extendoReturnFromBars || auraTransferEdgeCase || metaTransferEdgeCase ? OuttakeState.TRANSFER_START :
                                    OuttakeState.TRANSFER_END;
                    // we can skip the transfer start as the intake should come back at the transfer position coming from sample side
                    if (goToHPDeposit)
                    {
                        dropped = false;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    }
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                hpExtendoToggle();
                outtakeKeepTurretBack();
                extendoReturnFromBarsToggle(gamepad1.left_bumper || gamepad2.y);
                auraTransferToggle(gamepad2.dpad_right);
                metaTransferToggle(gamepad2.dpad_left);

                outtakeSubsystem.liftToInternalPID(0.4); // i cannot explain why this is necessary but it improved the transfer consistency...
                if (gamepad1.right_stick_button)
                {
                   intakeSubsystem.intakeSlideMotorRawControl(0);
                   intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                }
                else
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    if (intakeEdgeCase) // this case we want to make sure the arm is up before turning the turret
                    {
                        if (intakingFromBars) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
                        else if (delay(150))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        }
                        if (delay(300))
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        }
                        if (delay(450)) intakeSubsystem.intakeSlideMotorRawControl(-1); //intakeClipHoldLogic(-10, 5);
                        else intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget + 5); // we need to go a little bit forward to clear the battery
                    }
                    else
                    {
                        if (extendoReturnFromBars)
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            if (delay(30))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
                            }
                            if (delay(100)) // gives time for the arm to go up
                            {
                                intakeSubsystem.intakeSlideMotorRawControl(-1);
                            }
                        } else
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
//                            }
//                            else
//                            {
//                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
//                                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
//                                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT);
//                                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
//                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT);
//                            }
                            if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                            {
                                intakeClipHoldLogic(-10, 5);
                            } else intakeSubsystem.intakeSlideMotorRawControl(-1);
                        }
                    }
                }
                break;
            case INTAKE:
                if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1.left_bumper);
                    if (!auraTransferEdgeCase) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);

                    resetTimer();
                    break;
                }
                if (toggleIntakeEdgeCase.mode(gamepad2.dpad_down || gamepad1.dpad_down))
                {
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    intakeEdgeCase = !intakeEdgeCase;
                    internalTimerReset();
                }
                hpExtendoToggle();
                autoSampleOuttakeToggle(gamepad1.a);
                outtakeKeepTurretBack();
                auraTransferToggle(gamepad2.dpad_right);
                metaTransferToggle(gamepad2.dpad_left);

                if (!intakeEdgeCase)
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    if (delay(120)) // waits until the arm is out
                    {
                        intakeTurretPresetsOrNot();
                    }
                    intakeClipHoldLogic(slideTeleBase, 5);
                }
                else
                {
                    if (internalDelay(50))
                    {
                        intakeSlideTarget = 7;
                        if (delay(100)) // doesn't hit the clip
                            intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.AROUND);
                        if (internalDelay(320))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.BACK);
                        }
                        else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                    }
                }

                if (gamepad1.right_stick_button) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                colourValue = intakeSubsystem.getColorValue();
                if (
                        (delay(200) && colourValue >= sampleThreshold) ||
                        (gamepad1.left_stick_button)
                )
                {

                    state = !intakeEdgeCase && !goToHPExtendoDeposit? OuttakeState.TRANSFER_START : OuttakeState.AFTER_EXTENDO;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    gamepad1.rumbleBlips(2);
                    isSample = true;
                    resetTimer();
                    break;
                }
                break;
            case TRANSFER_START: // this state is skipped when coming from the extendo and not from the bars
                if ((
                        (auraTransferEdgeCase || metaTransferEdgeCase ? delay(550) : delay(335) &&
                        outtakeSubsystem.liftAtBase() &&
                        intakeSubsystem.isSlidesAtBase()) || (delay(700)))
                )
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                    break;
                }

//                presetChaining(gamepad1.a, false, gamepad1.x, gamepad1.y);

//                intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 8); // this controls the intake slides and the clip
                if (delay(300)) intakeSubsystem.intakeSlideMotorRawControl(0);
                else intakeSubsystem.intakeSlideMotorRawControl(-0.5);

                outtakeSubsystem.liftToInternalPID(-0.5);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
//                else outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
                if (!metaTransferEdgeCase)
                {
                    if (delay(20))
                    {
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);

                        if (!auraTransferEdgeCase)
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        } else
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_AURA);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_AURA);
//                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                            if (delay(150))
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AURA_TRANSFER);
                        }
                    }
                    if (delay(50))
                    {
                        outtakeKeepTurretBack();
                    }
                }
                else
                {
                    if (delay(50))
                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_META);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_META);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_META);
                    if (delay(200))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_META);

                    }
                    if (delay(300)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.TRANSFER_META);
                }
                break;
            case TRANSFER_END:
                // so this is when the claw will grip and we are assuming that the everything is at transfer position
                if (delay(200))
                {
                    // we want to remember if we went low
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeSlideMotorRawControl(0);
                    outtakeSubsystem.liftMotorRawControl(0);
                    if (isSample) state = OuttakeState.OUTTAKE_ADJUST;
                    else
                    {
                        state = OuttakeState.SPECIMEN_DEPOSIT_BACK2;
                        dropped = false;
                        intaked = false;
                        intakingFromBars = true; // this always true in this state
                        extendoReturnFromBars = true;
                        intakeSlideTarget = 0;
                        intakeSlideBtn.OffsetTargetPosition = 0; // this makes sure the extendo doesn't go out when we go into the deposit state
                    }
                    resetTimer();
                    break;
                }
                intakeSubsystem.intakeSlideMotorRawControl(-0.5);
                if (!metaTransferEdgeCase)
                    outtakeKeepTurretBack();
                else outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_META);

                if ((delay(60))) // this delay is high because of the transfer_start skip
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(130))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    }
                }
                break;
            case SPECIMEN_INTAKE:
                if (internalDelay(300) && intaked)
                {
                    state = OuttakeState.SPECIMEN_DEPOSIT;
                    isSample = false;
                    dropped = false;
                    goToIntake = false;
                    intakingFromBars = true; // this always true in this state
                    extendoReturnFromBars = true; // this always true in this state
                    intakeSlideTarget = 0;
                    intakeSlideSubBtn.OffsetTargetPosition = 0; // this makes sure the extendo doesn't go out when we go into the deposit state
                    intakeTurretToggle.OffsetTargetPosition = 2;
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);
                if (!intaked)
                {
                    if (delay(400))
                    {
                        toggleAutoAlign();
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);

                        if (autoAlignSpecIntake)
                        {
                            if (isBetweenAngle(Angles.normalizeDegrees(heading),   -25, 25)) //heading <= 45 && heading >= -45),
                            {
                                outtakeSubsystem.turretKeepToAngleTicks(0.1, heading);
                            } else outtakeSubsystem.turretRawControl(0);
                        }
                        else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

                        liftFineAdjustIntakeBtn.upToggle(gamepad2.right_bumper);
                        liftFineAdjustIntakeBtn.downToggle(gamepad2.left_bumper, 1);
                        if (toggleFineAdjustLift.mode(gamepad2.right_bumper || gamepad2.left_bumper))
                        {
                            fineAdjustingLiftIntake = true;
                            liftFineAdjustIntakeCache = outtakeSubsystem.liftSpecimenIntakePos + liftFineAdjustIntakeBtn.OffsetTargetPosition; // inch offset
                        }
                        if (!fineAdjustingLiftIntake)
                            outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftSpecimenIntakePos);
                        else
                        {
                            outtakeSubsystem.liftToInternalPID(liftFineAdjustIntakeCache);
                        }
                    }
                    else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

                    if (delay(700) && toggleRisingEdge.mode(gamepad1.right_bumper))
                    {
                        internalTimerReset();
                        intaked = true;
                        break;
                    }
                }
                else
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (internalDelay(90))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                        if (internalDelay(150))
                        {
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                        }
                    }
                }
                break;
            case OUTTAKE_ADJUST: // this allows for the drivers to pre adjust heights
                if (
                        (gamepad1.right_bumper || gamepad2.right_bumper) && delay(100) ||
                                (autoSampleOuttake && delay(40))
                )
                {
                    state = OuttakeState.SAMPLE_DEPOSIT;
                    isSample = true;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                if (gamepad2LeftTrigger())
                {
                    state = OuttakeState.HP_DEPOSIT;
                    resetTimer();
                    break;

                }
                autoSampleOuttakeToggle(gamepad1.a);
                liftHeightLogic(gamepad2.x, gamepad2.a);
                pivotToggle();
                toggleAutoAlign();
                break;
            case HP_DEPOSIT:
                liftHeightLogic(gamepad2.x, gamepad2.a);
                if (!dropped)
                {
                    if (delay(300))
                    {
                        toggleAutoAlign();
                        outtakeSubsystem.liftToInternalPID(0);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);

                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.HP_DEPOSIT);

                        if (intakeSubsystem.isSlidesAtBase() || delay(1200)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HP_DEPOSIT);
                        else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
                        intakeSubsystem.intakeSlideInternalPID(-3);

                        if (autoAlignHpDeposit)
                        {
                            if (isBetweenAngle(Angles.normalizeDegrees(heading), -25, 25))
                            {
                                outtakeSubsystem.turretKeepToAngleTicks(0, heading);
                            } else outtakeSubsystem.turretRawControl(0);
                        }
                        else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                    }
                    else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

                    if (delay(420) && toggleRisingEdge.mode(gamepad1.right_bumper))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        secondToggleForTheDrop.mode(gamepad1.right_bumper);
                        intakeSlideTarget = 0;
                        intakeSlideSubBtn.OffsetTargetPosition = 0;
                        intakeArmToggle.OffsetTargetPosition = 0;
                        intakeTurretToggle.OffsetTargetPosition = 2;
                        fineAdjustingIntakeSlides = false;
                        dropped = true;
                        intaked = false;
                        resetTimer();
                        break;
                    }
                }
                else
                {
                    toggleAutoAlign();
                    if (gamepad1.right_stick_button) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    if (autoAlignHpDeposit)
                    {
                        if (isBetweenAngle(Angles.normalizeDegrees(heading), -25, 25))
                        {
                            outtakeSubsystem.turretKeepToAngleTicks(0, heading);
                        } else outtakeSubsystem.turretRawControl(0);
                    }
                    else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

                    if (!intaked)
                    {
                        if (delay(250)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        if (delay(400)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                        if (delay(450) && secondToggleForTheDrop.mode(gamepad1.right_bumper))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            intaked = true;
                            resetTimer();
                            break;
                        }
                    }
                    else if (delay(200))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                        state = OuttakeState.SPECIMEN_DEPOSIT;
                        intakeSlideTarget = 0;
                        intakeSlideSubBtn.OffsetTargetPosition = 0;
                        dropped = false;
                        resetTimer();
                        break;
                    }
                }

                break;
            case HP_DEPOSIT_EXTENDO:
                if (delay(200))
                {
                    if (!extended)
                    {
                        intakeSlideTarget = -1;
                        if (toggleRisingEdge.mode(gamepad1.right_bumper))
                        {
                            secondToggleForTheDrop.mode(gamepad1.right_bumper);
                            extended = true;
                        }
                    } else
                    {
                        if (secondToggleForTheDrop.mode(gamepad1.right_bumper))
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                            dropped = true;
                            internalTimerReset();
                        }
                        intakeSlideTarget = 23.2;
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                    }
                    if (dropped && internalDelay(400))
                    {
                        state = OuttakeState.RETURN;
                        resetTimer();
                        break;
                    }
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                }
                else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FINISH); // this runs so we don't hit the outtake and i we have a visual clue of the state

                if (internalDelay(450) && intakeSubsystem.getColorValue() < 150 && dropped)

                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                break;
            case SAMPLE_DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if (delay(500) && dropped) //secondToggleForTheDrop.mode(gamepad1.right_bumper) && delay(400) && dropped)
                {
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);
                pivotToggle();
                autoSampleOuttakeToggle(gamepad1.a);
                if (!dropped)
                {
                    if (delay(40))
                    {
                        if (Math.abs(gamepad2.right_stick_y) > 0)
                        {
                            fineAdjustingLiftSample = true;
                        }
                        if (fineAdjustingLiftSample)
                        {
                            outtakeSubsystem.liftMotorRawControl(-gamepad2.right_stick_y);
                        }
                        else
                        {
                            outtakeLiftPresets(); // this just runs the correct height for the lift
                        }

                        toggleAutoAlign();
                        if (!isSampleLow ? outtakeSubsystem.liftPosition > outtakeSubsystem.liftTarget * 0.63 || delay(500) :
                        delay(100))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                        }

                        if (delay(300) )
                        {
                            if (autoAlignSampleDeposit)
                            {
                                if (isBetweenAngle(Angles.normalizeDegrees(heading), -90, 90)) //heading <= 45 && heading >= -45),
                                {
                                    outtakeSubsystem.turretKeepToAngleTicks(-135, heading);
                                } else outtakeSubsystem.turretRawControl(0);
                            }
                            else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                        }
//                        else turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.SAMPLE,
//                                OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                        if (delay(200))
                        {
                            outtakeSubsystem.pivotSetPosByOrder(togglePivot.OffsetTargetPosition);
                        }

                    }
                    if (delay(200) && toggleRisingEdge.mode(gamepad1.right_bumper))
                    {
//                        secondToggleForTheDrop.mode(gamepad1.right_bumper);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                        dropped = true;
                        resetTimer();
                    }
                }
                break;
            case SPECIMEN_DEPOSIT: // we want to deposit and intake

                //presetChaining(false, gamepad1LeftTrigger(),  gamepad1RightTrigger(), gamepad1.a);
                toggleAutoAlign();
                if (!dropped)
                {
//                    liftHeightLogic(gamepad2.x, gamepad2.a);
                    isSpecimenLow = false; //FIXME:
                    if (delay(40))
                    {
                        if (autoAlignSpecDeposit)
                        {
                            if (isBetweenAngle(Angles.normalizeDegrees(heading), -60, 65)) //heading <= 45 && heading >= -45),
                            {
                                outtakeSubsystem.turretKeepToAngleTicks(0.1, heading);
                            } else outtakeSubsystem.turretRawControl(0);
                        }
                        else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
/*
                        liftFineAdjustBtn.upToggle(gamepad1RightTrigger());
                        liftFineAdjustBtn.downToggle(gamepad1LeftTrigger(), 1);
                        if (toggleFineAdjustLift.mode(false && gamepad1RightTrigger() && gamepad1LeftTrigger())) // this could maybe be simplified, but this construction is more logical, test and maybe do
                        {
                            if (wasSpecLow != isSpecimenLow)
                            {
                                liftFineAdjustBtn.OffsetTargetPosition =
                                        isSpecimenLow ? liftFineAdjustSpecLowCache : liftFineAdjustSpecHighCache; // basically this should ensure we can flip between high and low
                                wasSpecLow = isSampleLow;
                            }
                            liftTarget = (isSpecimenLow ? outtakeSubsystem.liftLowBarPos : outtakeSubsystem.liftHighBarPos) + liftFineAdjustBtn.OffsetTargetPosition; // this will increment at one inch
                            if (isSpecimenLow) liftFineAdjustSpecLowCache = liftFineAdjustBtn.OffsetTargetPosition;
                            else liftFineAdjustSpecHighCache = liftFineAdjustBtn.OffsetTargetPosition;
                        } else if (fineAdjustingLiftSpec) // this should work
                        {
                            liftTarget = isSpecimenLow ? outtakeSubsystem.liftLowBarPos + liftFineAdjustSpecLowCache :
                                    outtakeSubsystem.liftHighBarPos + liftFineAdjustSpecHighCache;
                        }
                        else
                        {
                            liftTarget = isSpecimenLow ? outtakeSubsystem.liftLowBarPos : outtakeSubsystem.liftHighBarPos;
                        }*/
                        outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos);
                        outtakeSubsystem.wristState(isSpecimenLow ? OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_LOW :
                                OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.armState(isSpecimenLow ? OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_LOW :
                                OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.wristState(isSpecimenLow ? OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_LOW :
                                OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                        specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.right_bumper);
                        if (toggleRisingEdge.mode(gamepad1.right_bumper))
                        {
                            secondToggleForTheDrop.mode(gamepad1.right_bumper);
                            dropped = true;
                            internalTimerReset();
                            break;
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    if (internalDelay(250))
                    {
                        if (secondToggleForTheDrop.mode(gamepad1.right_bumper))
                        {
                            state = OuttakeState.RETURN;
                            resetTimer();
                            break;
                        }
                        if (!intakeEdgeCase) // this will cause a bug because intake edge case resets the internal timer
                        {

                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                            outtakeSubsystem.liftToInternalPID(0);

                        }
//                        outtakeKeepTurretBack(); // this has to be outside the if statement or else the pid wont update
                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                    }
                }

                intakeSlideSubBtn.upToggle(gamepad2.right_bumper);
                intakeSlideSubBtn.downToggle(gamepad2.left_bumper, 1);

                if (Math.abs(gamepad2.left_stick_y) > 0) fineAdjustingIntakeSlides = true;
                if (toggleRisingEdgeD2Intake.mode(gamepad2.left_bumper || gamepad2.right_bumper))
                {
                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
                    fineAdjustingIntakeSlides = false;
                }
                else if (fineAdjustingIntakeSlides)
                {
                    intakeSubsystem.intakeSlideMotorRawControl(-gamepad2.left_stick_y * 0.8);
                }
                else if (delay(110))
                {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                }
                colourValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);

                // Intake part
                if (delay(110))
                {

                    hpExtendoToggle();

                    if (toggleIntakeEdgeCase.mode(gamepad2.dpad_down || gamepad1.dpad_down))
                    {
                        intakeEdgeCase = !intakeEdgeCase;
                        if (!intakeEdgeCase)
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                        }
                        internalTimerReset();
                    }

                    if (!intakeEdgeCase)
                    {
                        if (internalDelay(300))
                        {
                            intakeSubArmHeight();
                            intakeTurretPresetsOrNot();
                        }
                    }
                    else
                    {
                        if (internalDelay(300))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                        }
                        else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.AROUND);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }

                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else if (intakeArmToggle.OffsetTargetPosition > 1)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (
                            (delay(300) && colourValue >= sampleThreshold || gamepad1.left_stick_button)
                    )
                    {
                        state = OuttakeState.HP_DEPOSIT;
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HP_DEPOSIT); // this makes sure arm is up
                        dropped = false;
                        toggleRisingEdge.mode(gamepad1.right_bumper);
                        gamepad1.rumbleBlips(1);
                        isSample = true;
                        resetTimer();
                        break;
                    }
                }
                else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                break;
            case SPECIMEN_DEPOSIT_BACK:
                liftHeightLogic(gamepad2.x, gamepad2.a);
                //presetChaining(false, gamepad1LeftTrigger(),  gamepad1RightTrigger(), gamepad1.a);
                toggleAutoAlign();

                if (autoAlignSpecDepositBack)
                {
                    if (isBetweenAngle(Angles.reduceDegrees(Math.toDegrees(heading)), 180, 360)) //heading <= 45 && heading >= -45),
                    {
                        outtakeSubsystem.turretKeepToAngleTicks(180, heading);
                    }
                    else outtakeSubsystem.turretRawControl(0);
                }
                else outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

                if (!dropped)
                {
                    if (delay(40))
                    {
                        outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarBackStaticPos);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK_STATIC);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_STATIC);

                        specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.right_bumper);
                        if (specOnLeft) outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.LEFT);
                        else outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);

                        if (toggleRisingEdge.mode(gamepad1.right_bumper))
                        {
                            dropped = true;
                            internalTimerReset();
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_FLICK);
                    if (internalDelay(150)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    if (internalDelay(300))
                    {
                        state = OuttakeState.RETURN;
                        resetTimer();
                        break;
                    }
                }
                break;
            case SPECIMEN_DEPOSIT_BACK2:
                liftHeightLogic(gamepad2.x, gamepad2.a);
                //presetChaining(false, gamepad1LeftTrigger(),  gamepad1RightTrigger(), gamepad1.a);
                toggleAutoAlign();

                if (autoAlignSpecDepositBack)
                {
                    if (delay(200))
                    {
                        if (isBetweenAngle(Angles.reduceDegrees(heading), 180, 270)) //heading <= 45 && heading >= -45),
                        {
                            outtakeSubsystem.turretKeepToAngleTicks(0, heading);
                            turretAngleCached = false;
                        } else if (!turretAngleCached)
                        {
                            turretLastAngle = outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition);
                            turretAngleCached = true;
                        }
                        else outtakeSubsystem.turretSpinToGains(turretLastAngle);
                    } else outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                }
                else outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);

                if (!dropped)
                {
                    if (delay(40))
                    {
                        if (!autoAlignSpecDepositBack)// this keeps us inside the extension limit
                        {
                            outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarBackStaticPos);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK_STATIC);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_STATIC);
                        }
                        else
                        {
                            outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarBackKineticPos);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK_KINETIC);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_KINETIC);
                        }
                        specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.right_bumper);
                        if (specOnLeft) outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.LEFT);
                        else outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);

                        if (toggleRisingEdge.mode(gamepad1.right_bumper))
                        {
                            dropped = true;
                            internalTimerReset();
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.liftToInternalPID(14);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    if (internalDelay(200))
                    {
                        state = OuttakeState.RETURN;
                        resetTimer();
                        break;
                    }
                }
                break;
            case HANG_START:
                if (gamepad1.right_bumper && delay(100))
                {
                    state = OuttakeState.HANG_LEVEL2;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                outtakeSubsystem.liftMotorRawControl(1);
                outtakeSubsystem.turretRawControl(0);
//                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.CLIMB_START);
                break;
            case HANG_LEVEL2:
                if (toggleRisingEdge.mode(gamepad1.right_bumper) && delay(500))
                {
                    state = OuttakeState.HANG_LEVEL3;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
//                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                if (delay(300))
                {
                    driveBase.PTOState(DriveBaseSubsystem.PTOState.OUT);
                }
                if (delay(400))
                {
                    driveBase.ptoMotorsSetPower(-0.2);
                }
                break;
            case HANG_LEVEL3:
                if (toggleRisingEdge.mode(gamepad1.right_bumper) && delay(200))
                {
                    state = OuttakeState.HANG_END;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                driveBase.PTOState(DriveBaseSubsystem.PTOState.OUT);
                if (delay(500))
                {
                    outtakeSubsystem.liftMotorRawControl(-1);
                    driveBase.ptoMotorsSetPower(1);
                }
                break;
            case HANG_END:
                outtakeSubsystem.lockServoState(OuttakeSubsystem.OuttakeLockServoState.LOCKED);
                if (delay(120))
                {
                    driveBase.PTOState(DriveBaseSubsystem.PTOState.IN);
                    outtakeSubsystem.liftMotorRawControl(0);
                    driveBase.ptoMotorsSetPower(0);
                }
                if (delay(300))
                {
                    outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                }
                break;
            case RETURN:
                if ((outtakeSubsystem.liftAtBase() && intakeSubsystem.isSlidesAtBase() && delay(300))
                        || delay(1350))
                {
                    isSample = true;
                    goToIntake = false;
                    goToSampleDeposit = false;
                    goToHPDeposit = false;
                    goToHPExtendoDeposit = false;
                    dropped = false;
                    intaked = false;
                    intakeTurretUsingPresets = false; // removed this so i only have to toggle it off once
                    intakeEdgeCase = false;
                    intakingFromBars = false;
                    fineAdjustingLiftSample = false;
                    armWentDown = false;
                    extendoReturnFromBars = false; // ig this should not be remembered
                    auraTransferEdgeCase = false; // all edge cases should not be remembered
                    metaTransferEdgeCase = false;

                    extended = false;
                    specOnLeft = true;
                    fineAdjustingIntakeSlides = false;
                    turretAngleCached = false;
                    // we want to remember auto aligns as they are different for each
                    outtakeSubsystem.liftMotorRawControl(0);
                    intakeSubsystem.intakeSlideMotorRawControl(0);
                    intakeSlideTarget = 0;
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    intakeArmToggle.OffsetTargetPosition = 0;
                    togglePivot.OffsetTargetPosition = 2;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                    state = OuttakeState.READY;
                    resetTimer();
                    break;
                }
                //intakeClipHoldLogic(slideTeleBase, 6);
                if (!intakeEdgeCase || delay(600))
                {
                    intakeSubsystem.intakeSlideMotorRawControl(-1);
                }
                outtakeSubsystem.liftMotorRawControl(-1);

                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);

                if (intakeSubsystem.isSlidesAtBase())
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
                }
                else if (!intakeEdgeCase) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
                if (delay(120))
                {
                    outtakeKeepTurretBack();
                    if (intakeEdgeCase)
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                        if (delay(300))
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    }
                    else intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                }
                break;
            case MANUAL_ENCODER_RESET:
                if (delay(200) && gamepad2.right_bumper)
                {
                    gamepad1.rumbleBlips(1);
                    outtakeSubsystem.liftMotorEncoderReset();
                    intakeSubsystem.intakeSlideMotorEncoderReset();
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                if (delay(100))
                {
                    outtakeSubsystem.liftMotorRawControl(-gamepad2.left_stick_y);
                    intakeSubsystem.intakeSlideMotorRawControl(-gamepad2.right_stick_y);
                    outtakeSubsystem.turretRawControl(gamepad2.right_trigger - gamepad2.left_trigger);
                }
                if (gamepad1.right_stick_button)
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                }
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);

                break;
        }
        // we run everything here that isn't state specific
        if ((
                (gamepad1.b || (gamepad2.b && !gamepad2.options)) &&
                (state != OuttakeState.READY) &&
                (state != OuttakeState.MANUAL_ENCODER_RESET) &&
                (state != OuttakeState.RETURN)
                )
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
        if (gamepad1.dpad_up)
        {
            imu.resetYaw();
        }
        if (gamepad1.dpad_left)
        {
            // this should work maybe actually no fck clue bro
            outtakeSubsystem.resetTurretPosition();
        }
    }

    public void auraTransferToggle(boolean toggle)
    {
        if (toggleAuraTransfer.mode(toggle))
        {
            auraTransferEdgeCase = !auraTransferEdgeCase;
        }
    }
    public void metaTransferToggle(boolean toggle)
    {
        if (toggleMetaTransfer.mode(toggle))
        {
            metaTransferEdgeCase = !metaTransferEdgeCase;
        }
    }

    public void hpExtendoToggle()
    {
        if (hpExtendoToggle.mode(gamepad1.y))
        {
            goToHPExtendoDeposit = !goToHPExtendoDeposit;
            gamepad1.rumbleBlips(1);
        }
    }
    public void toggleAutoAlign()
    {
        if (autoAlignToggle.mode(gamepad1.x)) // this is such a simplified way of doing this lol
        {
            if (state == OuttakeState.HP_DEPOSIT)
            {
                autoAlignHpDeposit = !autoAlignHpDeposit;
            }
            else if (state == OuttakeState.SAMPLE_DEPOSIT)
            {
                autoAlignSampleDeposit = !autoAlignSampleDeposit;
            }
            else if (state == OuttakeState.SPECIMEN_DEPOSIT)
            {
                autoAlignSpecDeposit = !autoAlignSpecDeposit;
            }
            else if (state == OuttakeState.SPECIMEN_INTAKE)
            {
                autoAlignSpecIntake = !autoAlignSpecIntake;
            }
            else if (state == OuttakeState.SPECIMEN_DEPOSIT_BACK2)
            {
                autoAlignSpecDepositBack = !autoAlignSpecDepositBack;
            }
        }
    }
    public void updateHeading()
    {
            heading = Angles.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            angularVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

//    public void outtakeTurretToggle(boolean toggleBtn)
//    {
//        if (toggleOuttakeTurret.mode(toggleBtn))
//        {
//            transferFromFront = !transferFromFront;
//        }
//        if (transferFromFront)
//        {
//            turretTimer = globalTimer;
//            outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
//            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT);
//            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
//        }
//        else
//        {
//            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
//            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
//            if (globalTimer - turretTimer > 130)
//            {
//                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
//            }
//        }
//    }
    public void outtakeKeepTurretBack()
    {
        if (outtakeSubsystem.turretReached(180))
        {
            if (globalTimer - turretTimer > 100) // we can save battery if this motor is off
            {
                outtakeSubsystem.turretRawControl(0);
            } else
            {
                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
            }
        } else
        {
            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
            turretTimer = globalTimer;
        }
    }

    public void intakeTurretPresetsOrNot()
    {
        intakeTurretToggle.upToggle(gamepad2RightTrigger());
        intakeTurretToggle.downToggle(gamepad2LeftTrigger(), 1);
        if (intakeTurretModeToggle.mode(gamepad1.x) )
        {
            intakeTurretUsingPresets = !intakeTurretUsingPresets;
        }
        if (gamepad2RightTrigger() || gamepad2LeftTrigger())
        {
            intakeTurretUsingPresets = true;
        }
        if (intakeTurretUsingPresets)
        {
            if (intakeTurretToggle.OffsetTargetPosition == 0)
                turretServoState = IntakeSubsystem.IntakeTurretServoState.MAX_LEFT;
            if (intakeTurretToggle.OffsetTargetPosition == 1)
                turretServoState = IntakeSubsystem.IntakeTurretServoState.LEFT;
            if (intakeTurretToggle.OffsetTargetPosition == 2)
                turretServoState = IntakeSubsystem.IntakeTurretServoState.STRAIGHT;
            if (intakeTurretToggle.OffsetTargetPosition == 3)
                turretServoState = IntakeSubsystem.IntakeTurretServoState.RIGHT;
            if (intakeTurretToggle.OffsetTargetPosition == 4)
                turretServoState = IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT;
            intakeSubsystem.intakeTurret(turretServoState);
        }
        else intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
    }
    public void intakeSubArmHeight()
    {
        intakeArmToggle.upToggle(gamepad2.a);
        intakeArmToggle.downToggle(gamepad2.x, 1);
        if (gamepad1.dpad_right) intakeArmToggle.OffsetTargetPosition = 2;
        if (intakeArmToggle.OffsetTargetPosition == 0) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
        if (intakeArmToggle.OffsetTargetPosition == 1) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
        if (intakeArmToggle.OffsetTargetPosition == 2) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
    }
    public void  pivotToggle()
    {
        togglePivot.upToggle(gamepad2LeftTrigger());
        togglePivot.downToggle(gamepad2RightTrigger(), 1);
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
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jitter
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            intakeClipTimer = globalTimer;
        }
    }
//    public boolean colourLogicWithTimer()
//    {
//        if (intakeSubsystem.getColorValue() > 900) {
//            if (globalTimer - colourTimer > 50){
//                return  (prevIsYellow == intakeSubsystem.isYellow) && (prevIsRed == intakeSubsystem.isRed);
//            } else {
//                prevIsRed = intakeSubsystem.isRed;
//                prevIsYellow = intakeSubsystem.isYellow;
//            }
//        } else {
//            colourTimer = globalTimer;
//        }
//    }
    public void outtakeLiftPresets()
    {
        if (isSample)
        {
            if (isSampleLow)
            {
                outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftLowBucketPos);
            }
            else
            {
//                outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBucketPos);
                outtakeSubsystem.liftMotorRawControl(1);
            }
        }
        else
        {
            if (isSpecimenLow)  outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftLowBarPos);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else  outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
        }
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
    public void specOrSampleLogic(boolean spec, boolean sample)
    {
        if (sample) isSample = true;
        else if (spec) isSample = false;
    }
    public void specSideLogic(boolean left, boolean right)
    {
        if (left) specOnLeft = true;
        else if (right) specOnLeft = false;
    }
    public void presetChaining(boolean goToSampleDeposit, boolean goToIntake, boolean goToHPDeposit, boolean goToHPExtendoDeposit)
    {
        if (goToSampleDeposit) this.goToSampleDeposit = true;
        if (goToIntake) this.goToIntake = true;
        if (goToHPDeposit) this.goToHPDeposit = true;
        if (goToHPExtendoDeposit) this.goToHPExtendoDeposit = true;
    }
    public void autoSampleOuttakeToggle(boolean toggle)
    {
        if (toggleAutoSampleOuttake.mode(toggle))
        {
            autoSampleOuttake = !autoSampleOuttake;
            gamepad1.rumbleBlips(1);
        }
    }
    public void extendoReturnFromBarsToggle(boolean toggle)
    {
        if (toggleExtendoReturningFormBars.mode(toggle))
        {
            intakingFromBars = true;
            extendoReturnFromBars = true;
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
        }
    }
//    public void selectDepositType(boolean goToSampleDeposit, boolean goToHPDeposit, boolean goToHPExtendoDeposit)
//    {
//        if (goToSampleDeposit)
//        {
//            this.goToSampleDeposit = true;
//            this.goToHPDeposit = false;
//            this.goToHPExtendoDeposit = false;
//        }
//        else if (goToHPDeposit)
//        {
//            this.goToSampleDeposit = false;
//            this.goToHPDeposit = true;
//            this.goToHPExtendoDeposit = false;
//        }
//        else if (goToHPExtendoDeposit)
//        {
//            this.goToSampleDeposit = false;
//            this.goToHPDeposit = false;
//            this.goToHPExtendoDeposit = true;
//        }
//    }
//    @Deprecated
//    public void turretSpinTo(double targetAngle, OuttakeSubsystem.OuttakeArmServoState armState, OuttakeSubsystem.OuttakeWristServoState wristState)
//    { // this currently has power cutoff
//        if (outtakeSubsystem.turretReached(targetAngle))
//        {
//            if (globalTimer - turretTimer > 150)
//            {
//                if (armState != null) // i just did this if i don't really want to pass a state the correct would be to pass default in the state
//                    outtakeSubsystem.armState(armState);
//                if (wristState != null)
//                    outtakeSubsystem.wristState(wristState);
//                outtakeSubsystem.turretRawControl(0);
//            }
//            else
//            {
//                outtakeSubsystem.turretSpinTo(targetAngle);
//            }
//        }
//        else
//        {
//            //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPIN);
//            //outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPIN);
//            outtakeSubsystem.turretSpinTo(targetAngle); // we
//            globalTimer = turretTimer;
//        }
//    }
    public boolean isBetweenAngle(double angle, double min, double max)
    {
        return angle <= max && angle >= min;
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
    public boolean internalDelay(double delayTime)
    {
        return globalTimer - internalTimer > delayTime;
    }
    public void internalTimerReset()
    {
        internalTimer = globalTimer;
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
