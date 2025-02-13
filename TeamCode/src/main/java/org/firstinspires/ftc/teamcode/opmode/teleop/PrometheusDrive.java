package org.firstinspires.ftc.teamcode.opmode.teleop;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideExtensionLimit;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleClose;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.accessory.SideAfterAuto;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownWithLimit;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "PrometheusDrive", group = "A")
public class PrometheusDrive extends LinearOpMode
{
    ElapsedTime GlobalTimer;
    double globalTimer, sequenceTimer, intakeClipTimer, turretTimer, internalTimer;;
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
        HANG_START,
        HANG_FIRST,
        HANG_END,
        RETURN,
        MANUAL_ENCODER_RESET,
    }
    OuttakeState state = OuttakeState.READY;
    ToggleUpOrDown intakeSlideBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDownWithLimit intakeSlideSubBtn = new ToggleUpOrDownWithLimit(1, 1, 0, 4); // this instance will control
    ToggleUpOrDown liftFineAdjustBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleUpOrDown liftFineAdjustIntakeBtn = new ToggleUpOrDown(1, 1, 0);
    ToggleRisingEdge toggleRisingEdge = new ToggleRisingEdge();
    ToggleRisingEdge toggleRisingEdgeD2Intake = new ToggleRisingEdge();
    ToggleRisingEdge secondToggleForTheDrop = new ToggleRisingEdge();
    ToggleRisingEdge toggleFineAdjustLift = new ToggleRisingEdge();
    ToggleRisingEdge toggleIntakeEdgeCase = new ToggleRisingEdge();
    ToggleRisingEdge toggleOuttakeTurret = new ToggleRisingEdge();
    ToggleUpOrDown intakeArmToggle = new ToggleUpOrDown(1, 1,0);
    ToggleUpOrDown intakeTurretToggle = new ToggleUpOrDown(1, 1, 2);
    int intakeSLideIncrement = 5; // in
    double colorValue;
    double intakeSlideTarget;
    double intakeSlideTargetClimb = 17;
    double liftTarget;
    boolean intakeEdgeCase = false;
    boolean dropped = false;
    boolean isSampleLow = false, isSpecimenLow = false; // this start at high and remembers
    boolean wasSpecLow = false, wasSampleLow = false;
    boolean isSample = false;
    boolean specOnLeft;
    boolean goToIntake = false;
    boolean goToSampleDeposit = false;
    boolean goToHPDeposit = false;
    boolean goToHPExtendoDeposit;
    boolean fineAdjustingLiftIntake = false;
    double fineAdjustLiftIntakeCache;
    boolean fineAdjustingRailManualReset = false;
    boolean intakeTurretUsingPresets = false;
    boolean fineAdjustingLiftSpec = false;
    int liftFineAdjustSpecLowCache;
    int liftFineAdjustSpecHighCache;
    double liftFineAdjustIntakeCache;
    boolean transferFromFront;
    boolean lowBarTransfer;
    boolean intaked;

    IMU imu;
    double angularVel, heading;


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
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.OFF;
            outtakeSubsystem = new OuttakeSubsystem(hardware);
            driveBase = new DriveBaseSubsystem(hardware);
            prevIntakeFilterState = intakeSubsystem.intakeFilter;
            driveBase.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            imu = hardware.imu;
            //imu.initialize()
            imu.resetYaw();


            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);

            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
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
                    state == OuttakeState.HP_DEPOSIT_EXTENDO ||
                    state == OuttakeState.INTAKE_EXTENDO_SUB ||
                    state == OuttakeState.SPECIMEN_DEPOSIT
            );
            outtakeSubsystem.outtakeReads();
            if (
                    state == OuttakeState.INTAKE_EXTENDO_SUB ||
                    state == OuttakeState.SPECIMEN_INTAKE ||
                    state == OuttakeState.SPECIMEN_DEPOSIT )
            {
                heading = Angles.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                angularVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate;
            }
            outtakeSequence();


            telemetry.addData("State", state);
            telemetry.addData("Side", hardware.side);
//            telemetry.addData("Side after auto", SideAfterAuto.side);
//            telemetry.addData("Go to intake", goToIntake);
//            telemetry.addData("Go to Deposit", goToDeposit);
//            telemetry.addData("Go to HpDeposit", goToHPDeposit);
//            telemetry.addData("IsArmOver", outtakeSubsystem.isArmOver());
            telemetry.addData("FilterState", intakeSubsystem.intakeFilter);
            telemetry.addData("Color logic", intakeSubsystem.colorLogic());
            telemetry.addData("IsRed", intakeSubsystem.isRed);
            telemetry.addData("IsYellow", intakeSubsystem.isYellow);
            telemetry.addData("Color value", intakeSubsystem.getColorValue());
//            telemetry.addData("OuttakePos", outtakeSubsystem.liftPosition);
//            telemetry.addData("IntakePos", intakeSubsystem.slidePosition);
//            telemetry.addData("Dropped", dropped);
//            telemetry.addData("ClimbPos", driveBase.getClimbPosition());
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
                if (gamepad1.right_bumper)
                {
                    state = OuttakeState.INTAKE;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                } else if (gamepad1.left_bumper)
                {
                    state = OuttakeState.INTAKE_EXTENDO;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1.left_bumper);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                } else if (gamepad1RightTrigger())
                {
                    state = OuttakeState.INTAKE_EXTENDO_SUB;
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    intakeSlideTarget = 5;
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    intakeSlideSubBtn.upToggle(gamepad2.right_bumper);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                }
                else if (gamepad1LeftTrigger())
                {
                    state = OuttakeState.INTAKE_EXTENDO_SPEC;
                    intakeSlideTarget = slideTeleClose;
                    intakeSlideBtn.upToggle(gamepad1LeftTrigger());
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                }

                if (gamepad1.options)
                {
                    state = OuttakeState.HANG_START; // i need to improve this sequencing
                    resetTimer();
                    break;
                }
                if (gamepad2LeftTrigger())
                {
                    state = OuttakeState.SPECIMEN_INTAKE;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    transferFromFront = true;
                    resetTimer();
                    break;
                }

                if (gamepad1.b || gamepad2.b) // might have the slides go out for a bit to get stuff unstuck maybe the arm goes forward too?
                {
                    intakeSubsystem.intakeSpin(-1);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                }
                else
                {
                    intakeClipHoldLogic(0, 8);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                }
                break;
            case INTAKE_EXTENDO: // this state is just for using the extendo outside of the submersible
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;


                if (delay(50))
                {
                    colorValue = intakeSubsystem.getColorValue();
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);


                    if (delay(120)) // waits until the arm is out
                    {
                        intakeTurretToggle.upToggle(gamepad1RightTrigger());
                        intakeTurretToggle.downToggle(gamepad1LeftTrigger(), 2);

                        if (gamepad1LeftTrigger() || gamepad1RightTrigger())
                        {
                            intakeTurretUsingPresets = true;
                        }
                        if (intakeTurretUsingPresets)
                        {
                            double angle = 25 * intakeTurretToggle.OffsetTargetPosition;
                            if (angle >= 20 && angle <= 150)
                            {
                                intakeSubsystem.intakeTurretSetAngle(25 * intakeTurretToggle.OffsetTargetPosition);
                            }
                        }
                        else intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
                    }

                    if ((delay(700) && colorValue > 500) ||
                            gamepad1.y)
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
                intakeSlideBtn.upToggle(gamepad1RightTrigger());
                intakeSlideBtn.downToggle(gamepad1LeftTrigger(), 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;

                if (delay(50))
                {
                    colorValue = intakeSubsystem.getColorValue();
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);
                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.dpad_left);
                    if (toggleOuttakeTurret.mode(gamepad1.a))
                    {
                        transferFromFront = !transferFromFront;
                    }
                    if (transferFromFront) turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    else turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);

                    if (delay(120)) // waits until the arm is out
                    {
                        intakeTurretToggle.upToggle(gamepad1.left_bumper);
                        intakeTurretToggle.downToggle(gamepad1.right_bumper, 2);

                        if (gamepad1.left_bumper || gamepad1.right_bumper)
                        {
                            intakeTurretUsingPresets = true;
                        }
                        if (intakeTurretUsingPresets)
                        {
                            double angle = 25 * intakeTurretToggle.OffsetTargetPosition;
                            if (angle >= 20 && angle <= 150)
                            {
                                intakeSubsystem.intakeTurretSetAngle(25 * intakeTurretToggle.OffsetTargetPosition);
                            }
                        }
                        else intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
                    }

                    if ((delay(700) && colorValue > 500) ||
                            gamepad1.y)
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
            case INTAKE_EXTENDO_SUB:
                // the intake part
                intakeSlideSubBtn.upToggle(gamepad2.right_bumper);
                intakeSlideSubBtn.downToggle(gamepad2.left_bumper, 1);
                if (toggleRisingEdgeD2Intake.mode(gamepad2.left_bumper || gamepad2.right_bumper))
                {
                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
                } else
                {
                    intakeSlideTarget += -gamepad2.right_stick_y * 0.8; // 0.8 in (20 in per second) // FIXME: Should probably normalize this with looptimes
                    if (intakeSlideTarget > slideExtensionLimit)
                        intakeSlideTarget = slideExtensionLimit; // caps for extension limit
                }
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                selectDepositType(gamepad1.a, gamepad1.x, gamepad1.y);
                if (goToSampleDeposit) transferFromFront = false;
                else if (goToHPDeposit) transferFromFront = true;

                if (gamepad1LeftTrigger())
                {
                    lowBarTransfer = true;
                }
                if (delay(110))
                {
                    if (toggleOuttakeTurret.mode(gamepad1.a))
                    {
                        transferFromFront = !transferFromFront;
                    }
                    if (transferFromFront) turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    else turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);

                    intakeSubArmHeight();
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);

                    if (gamepad2LeftTrigger() || gamepad2RightTrigger())
                    {
                        intakeTurretUsingPresets = true;
                    }
                    if (intakeTurretUsingPresets) intakeSubsystem.intakeTurretSetAngle(25 * intakeTurretToggle.OffsetTargetPosition);
                    else intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
                    if (gamepad2.dpad_down || gamepad1.dpad_up) intakeTurretUsingPresets = false;

                    if (gamepad1.right_stick_button)
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else  // this makes so the brushes only run after we have dropped
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                    if (
                            (delay(300) && colorValue > 900 || gamepad1.left_stick_button)
                    )
                    {
                        if (!gamepad1.left_stick_button && (intakeSubsystem.colorLogic() || gamepad1.y))
                        {
                            state = OuttakeState.AFTER_EXTENDO;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            if (intakeArmToggle.OffsetTargetPosition < 0) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.IN);
                            else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL); // this makes sure arm is up
                            gamepad1.rumbleBlips(1);
                            resetTimer();
                            break;
                        }
                    }
                }
                else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                break;
            case AFTER_EXTENDO: // this might be necessary to check the colour and go back the slides
                if (delay(200) && intakeSubsystem.isSlidesAtBase())
                {
                    state = goToHPExtendoDeposit ? OuttakeState.HP_DEPOSIT_EXTENDO : OuttakeState.TRANSFER_START;
                    isSample = intakeSubsystem.isSample();
                    resetTimer();
                    break;
                }
                presetChaining(gamepad1.a, false, gamepad1.x, gamepad1.y);
                if (gamepad1LeftTrigger())
                {
                    lowBarTransfer = true;
                }
                if (!goToHPExtendoDeposit && (goToSampleDeposit || goToHPDeposit)) // we don't need to turn the turret if the
                {
                    if (transferFromFront)
                        turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    else
                        turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                }

                if (gamepad1.right_stick_button)
                {
                   intakeSubsystem.intakeSlideMotorRawControl(0);
                   if (delay(200))
                   {
                       intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                   }
                   else intakeSubsystem.intakeSpin(0); // this might have to be spinning in to lock in
                }
                else
                {
                    intakeSubsystem.intakeSpin(0);
                    if (lowBarTransfer)
                    {
                        if (delay(320))
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.AROUND);
                        }
                        else if (delay(210))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.IN);
                        }
                        else if (delay(110))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AROUND);
                        }
                        else intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);

                    }
                    else
                    {
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        if (delay(100))
                        {
                            intakeSubsystem.intakeArm(transferFromFront ? IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT :
                                    IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        }
                    }
                    if (delay(200)) // gives time for the arm to go up
                    {
                        intakeClipHoldLogic(-10, 5);
                    }
                }
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
                if (toggleIntakeEdgeCase.mode(gamepad1.dpad_down))
                {
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    intakeEdgeCase = !intakeEdgeCase;
                    internalTimerReset();
                }

                if (!intakeEdgeCase)
                {
                    intakeTurretToggle.upToggle(gamepad1RightTrigger());
                    intakeTurretToggle.downToggle(gamepad1LeftTrigger(), 2);
                    if (delay(120)) // waits until the arm is out
                    {
                        if (gamepad1LeftTrigger() || gamepad1RightTrigger())
                        {
                            intakeTurretUsingPresets = true;
                        }
                        if (intakeTurretUsingPresets)
                            intakeSubsystem.intakeTurretSetAngle(25 * intakeTurretToggle.OffsetTargetPosition);
                        else intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
                    }
                    intakeClipHoldLogic(slideTeleBase, 5);
                }
                else
                {
                    if (internalDelay(100))
                    {
                        intakeSubsystem.intakeSlideInternalPID(7);
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        if (internalDelay(240))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.BACK);
                        }
                    }
                }

                if(gamepad1.right_stick_button) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                colorValue = intakeSubsystem.getColorValue();
                if (
                        (delay(400) && colorValue > 800 )||
                        (intakeSubsystem.intakeFilter == IntakeSubsystem.IntakeFilter.OFF && (toggleRisingEdge.mode(gamepad1.right_bumper))) ||
                        (gamepad2RightTrigger())

                )
                {
                    state = OuttakeState.TRANSFER_START;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    intakeTurretUsingPresets = false;
                    gamepad1.rumbleBlips(2);
                    resetTimer();
                    break;
                }
                break;

            case TRANSFER_START:
                // we should hold at transfer pos if nothing was selected
                if (
                        ((delay(290) && outtakeSubsystem.liftAtBase() && intakeSubsystem.isSlidesAtBase()) || delay(1000))
                        && (goToHPDeposit || goToSampleDeposit)) // we sit in this state until we have decided the continuation
                {
                    state = OuttakeState.TRANSFER_END;
                    resetTimer();
                    break;
                }
                if (delay(20) && goToHPExtendoDeposit) // we can exit this state early as we don't use the outtake in that state
                {
                    state = OuttakeState.HP_DEPOSIT_EXTENDO;
                    outtakeSubsystem.liftMotorRawControl(0);
                    resetTimer();
                    break;
                }
                presetChaining(gamepad1.a, false, gamepad1.x, gamepad1.y);

                intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                outtakeSubsystem.liftToInternalPID(-2);
                if (delay(50))
                {
                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    if (transferFromFront) turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT,
                            OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    else turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK,
                            OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);

                    if (delay(180)) intakeSubsystem.intakeArm(transferFromFront ?
                            IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT : IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                }
                break;
            case TRANSFER_END:
                // so this is when the claw will grip and we are assuming that the slides are at transfer position
                if (delay(490))
                {
                    // we want to remember if we went low
                    isSample = true;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    state = OuttakeState.OUTTAKE_ADJUST;
                    resetTimer();
                    break;
                }
                if ((delay(40)))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(150))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    }
                    if (delay(200))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
                    }
                }
                break;
            case SPECIMEN_INTAKE:
                if (internalDelay(300))
                {
                    state = OuttakeState.SPECIMEN_DEPOSIT;
                    isSample = false;
                    dropped = false;
                    goToIntake = false;
                    intakeSlideTarget = 0;
                    intakeSlideBtn.OffsetTargetPosition = 0; // this makes sure the extendo doesn't go out when we go into the deposit state
                    resetTimer();
                    break;
                }
                liftHeightLogic(gamepad2.x, gamepad2.a);
                if (!intaked)
                {
                    if (delay(60))
                    {
                        if (isBetweenAngle(outtakeSubsystem.turretAngle, -60, 60) && delay(200) || delay(600))
                        {
                            liftFineAdjustIntakeBtn.upToggle(gamepad1RightTrigger());
                            liftFineAdjustIntakeBtn.downToggle(gamepad1LeftTrigger(), 1);
                            if (toggleFineAdjustLift.mode(gamepad1RightTrigger() || gamepad1LeftTrigger()))
                            {
                                fineAdjustingLiftIntake = true;
                                liftFineAdjustIntakeCache = OuttakeSubsystem.liftSpecimenIntakePos + liftFineAdjustIntakeBtn.OffsetTargetPosition; // inch offset
                            }
                            if (!fineAdjustingLiftIntake)
                                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftSpecimenIntakePos);
                            else
                            {
                                outtakeSubsystem.liftToInternalPID(liftFineAdjustIntakeCache);
                            }
                        }
                        if (delay(300))
                        {
                            outtakeSubsystem.turretKeepToAngle(180, heading, true);
                        }
                        else turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.INTAKE, OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    }
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
                                (goToSampleDeposit && delay(40))
                )
                {
                    state = OuttakeState.SAMPLE_DEPOSIT;
                    toggleRisingEdge.mode(gamepad1.right_bumper);
                    resetTimer();
                    break;
                }
                specSideLogic(gamepad1.dpad_left || gamepad2.dpad_left, gamepad1.dpad_right || gamepad2.right_bumper);
                if ((gamepad1.left_bumper|| gamepad2.left_bumper) && delay(100))
                {
                    state = OuttakeState.SPECIMEN_DEPOSIT;
                }
                presetChaining(false, gamepad2LeftTrigger(), false, false);
                liftHeightLogic(gamepad2.x, gamepad2.a);
                break;
            case HP_DEPOSIT:
                if (delay(120) && dropped)
                {
                    state = goToIntake ? OuttakeState.SPECIMEN_INTAKE : OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                presetChaining(false, gamepad2LeftTrigger(), false, false);
                liftHeightLogic(gamepad2.x, gamepad2.a);
                if (!dropped)
                {
                    if (delay(40))
                    {
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.HP_DEPOSIT,
                                OuttakeSubsystem.OuttakeWristServoState.HP_DEPOSIT);
                    }
                }
                if (delay(420) && toggleRisingEdge.mode(gamepad1.right_bumper))
                {
                    dropped = true;
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    resetTimer();
                }
                break;
            case HP_DEPOSIT_EXTENDO:
                intakeSlideBtn.upToggle(gamepad1.left_bumper);
                intakeSlideBtn.downToggle(gamepad1.right_bumper, 1);
                if (intakeSlideBtn.OffsetTargetPosition == 1) intakeSlideTarget = slideTeleClose;
                if (intakeSlideBtn.OffsetTargetPosition == 2) intakeSlideTarget = slideTeleFar;

                if (delay(90))
                {

                    if (gamepad1.right_bumper && delay(200))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    }
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                    if (delay(200) || intakeSubsystem.getColorValue() < 500)
                    {
                        state = OuttakeState.RETURN;
                        resetTimer();
                        break;
                    }
                }

                break;
            case SAMPLE_DEPOSIT: // this will actually start the deposit, so lift and arm presets
                if (delay(250) && dropped) //secondToggleForTheDrop.mode(gamepad1.right_bumper) && delay(400) && dropped)
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
                        if (delay(300) )
                        {
                            if (isBetweenAngle(heading, -45, 45)) //heading <= 45 && heading >= -45),
                            {
                                outtakeSubsystem.turretKeepToAngle(135, heading);
                            }
                        }
                        else turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.SAMPLE, OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                        if (gamepad2LeftTrigger())
                        {
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        }
                        else if (gamepad2RightTrigger())
                        {
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        }

                    }
                    if (delay(200) && toggleRisingEdge.mode(gamepad1.right_bumper))
                    {
                        secondToggleForTheDrop.mode(gamepad1.right_bumper);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                        dropped = true;
                        resetTimer();
                    }
                }
                break;
            case SPECIMEN_DEPOSIT: // we want to deposit and intake

                liftHeightLogic(gamepad1.x, gamepad1.a);
                presetChaining(false, gamepad1LeftTrigger(),  gamepad1RightTrigger(), gamepad1.a);

                if (!dropped)
                {
                    if (delay(40))
                    {
                        // if
                        liftFineAdjustBtn.upToggle(gamepad1RightTrigger());
                        liftFineAdjustBtn.downToggle(gamepad1LeftTrigger(), 1);
                        if (toggleFineAdjustLift.mode(gamepad1RightTrigger() || gamepad1LeftTrigger())) // this could maybe be simplified, but this construction is more logical, test and maybe do
                        {
                            if (wasSpecLow != isSpecimenLow)
                            {
                                liftFineAdjustBtn.OffsetTargetPosition =
                                        isSpecimenLow ? liftFineAdjustSpecLowCache : liftFineAdjustSpecHighCache; // basically this should ensure we can flip between high and low
                                wasSpecLow = isSampleLow;
                            }
                            liftTarget = (isSpecimenLow ? OuttakeSubsystem.liftLowBarPos : OuttakeSubsystem.liftHighBarPos) + liftFineAdjustBtn.OffsetTargetPosition; // this will increment at one inch
                            if (isSpecimenLow) liftFineAdjustSpecLowCache = liftFineAdjustBtn.OffsetTargetPosition;
                            else liftFineAdjustSpecHighCache = liftFineAdjustBtn.OffsetTargetPosition;
                        } else if (fineAdjustingLiftSpec) // this should work
                        {
                            liftTarget = isSpecimenLow ? OuttakeSubsystem.liftLowBarPos + liftFineAdjustSpecLowCache :
                                    OuttakeSubsystem.liftHighBarPos + liftFineAdjustSpecHighCache;
                        }
                        else
                        {
                            liftTarget = isSpecimenLow ? OuttakeSubsystem.liftLowBarPos : OuttakeSubsystem.liftHighBarPos;
                        }
                        outtakeSubsystem.liftToInternalPID(liftTarget);
                        outtakeSubsystem.wristState(isSpecimenLow ? OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_LOW :
                                OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                        if (!intaked) // we know where it is if we picked from the wall
                        {
                            if (specOnLeft) outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                            else outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        }
                        if (toggleRisingEdge.mode(gamepad1.right_bumper))
                        {
                            secondToggleForTheDrop.mode(gamepad1.right_bumper);
                            dropped = true;
                            internalTimerReset();
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    if (internalDelay(110)) // reduce strain in the claw we wait
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                        if (internalDelay(140))
                        {
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPIN);
                        }
                    }
                }

                // the intake part
                intakeSlideSubBtn.upToggle(gamepad2.right_bumper);
                intakeSlideSubBtn.downToggle(gamepad2.left_bumper, 1);
                if (toggleRisingEdgeD2Intake.mode(gamepad2.left_bumper || gamepad2.right_bumper))
                {
                    intakeSlideTarget = intakeSLideIncrement * intakeSlideSubBtn.OffsetTargetPosition;
                }
                else
                {
                    intakeSlideTarget += -gamepad2.right_stick_y * 0.8; // 0.8 in (20 in per second) // FIXME: prob normalize this
                    if (intakeSlideTarget > slideTeleFar) intakeSlideTarget = slideTeleFar; // caps for extension limit
                }
                colorValue = intakeSubsystem.getColorValue();
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);

                if (delay(110))
                {
                    intakeTurretToggle.upToggle(gamepad2.right_bumper);
                    intakeTurretToggle.downToggle(gamepad2.left_bumper, 1);
                    if (intakeTurretUsingPresets)
                    {
                        intakeSubsystem.intakeTurretSetAngle(25 * intakeTurretToggle.OffsetTargetPosition);
                    }
                    else if (dropped) intakeSubsystem.intakeTurretBasedOnHeadingVel(angularVel);
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget);

                    if (dropped)
                    {
                        intakeSubArmHeight();
                        if (gamepad1.right_stick_button)
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        else
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }
                    else
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    }
                    if (
                            ((delay(300) && colorValue > 900) ||
                                    gamepad2.left_stick_button) &&
                                    dropped)
                    {
                        if (intakeSubsystem.colorLogic() || gamepad1.y)
                        {
                            state = OuttakeState.AFTER_EXTENDO;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL); // this makes sure arm is up
                            gamepad1.rumbleBlips(1);
                            dropped = false;
                            resetTimer();
                            break;
                        }
                    }
                }
                break;
            case HANG_START:

                break;
            case HANG_FIRST:

                break;
            case HANG_END:

                break;
            case RETURN:
                if ((outtakeSubsystem.liftAtBase() && delay(500) && !outtakeSubsystem.isArmOver()) || delay(950))
                {
                    isSample = true;
                    goToIntake = false;
                    goToSampleDeposit = false;
                    goToHPDeposit = false;
                    dropped = false;
                    outtakeSubsystem.liftMotorRawControl(0);
                    intakeSlideBtn.OffsetTargetPosition = 0;
                    intakeSlideTarget = 0;
                    intakeArmToggle.OffsetTargetPosition = 0;
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                    state = OuttakeState.READY;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(slideTeleBase, 6);
                outtakeSubsystem.liftToInternalPIDTicks(-100);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                if (delay(40))
                {
                    if (delay(120))
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    double diff = Angles.normalizeDegrees(outtakeSubsystem.turretAngle - 180);
                    if (diff <= 45 && diff>=-45)
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
                    }
                    else turretSpinTo(180 , OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK, OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                }
                break;
            case MANUAL_ENCODER_RESET:
                if (delay(200) && gamepad2.right_bumper)
                {
                    gamepad1.rumbleBlips(1);
                    outtakeSubsystem.liftMotorEncoderReset();
                    intakeSubsystem.intakeSlideMotorEncoderReset();
                    fineAdjustingRailManualReset = false;
                    state = OuttakeState.RETURN;
                    resetTimer();
                    break;
                }
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(100))
                {
                    outtakeSubsystem.liftMotorRawControl(-gamepad2.left_stick_y);
                    intakeSubsystem.intakeSlideMotorRawControl(-gamepad2.right_stick_y);
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
                (gamepad1.b || gamepad2.b) &&
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
        if (gamepad1.dpad_left)
        {
            imu.resetYaw();
        }
    }
    public void intakeArmHeight()
    {
        if (gamepad1.x || gamepad2.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
        else if (gamepad1.a || gamepad2.a) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
    }
    public void intakeSubArmHeight()
    {
        intakeArmToggle.upToggle(gamepad2.a);
        intakeArmToggle.downToggle(gamepad2.x, 1);
        if (intakeArmToggle.OffsetTargetPosition == -1) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.BACK);
        if (intakeArmToggle.OffsetTargetPosition == 0) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
        if (intakeArmToggle.OffsetTargetPosition == 1) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
        if (intakeArmToggle.OffsetTargetPosition == 2) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
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
            if (isSampleLow) outtakeSubsystem.liftToInternalPIDTicks(550);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(1655);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
        }
        else
        {
            if (isSpecimenLow)  outtakeSubsystem.liftToInternalPIDTicks(0);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(900);
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
    public void selectDepositType(boolean goToSampleDeposit, boolean goToHPDeposit, boolean goToHPExtendoDeposit)
    {
        if (goToSampleDeposit)
        {
            this.goToSampleDeposit = true;
            this.goToHPDeposit = false;
            this.goToHPExtendoDeposit = false;
        }
        else if (goToHPDeposit)
        {
            this.goToSampleDeposit = false;
            this.goToHPDeposit = true;
            this.goToHPExtendoDeposit = false;
        }
        else if (goToHPExtendoDeposit)
        {
            this.goToSampleDeposit = false;
            this.goToHPDeposit = false;
            this.goToHPExtendoDeposit = true;
        }
    }
    public void turretSpinTo(double targetAngle, OuttakeSubsystem.OuttakeArmServoState armState, OuttakeSubsystem.OuttakeWristServoState wristState)
    { // this currently has power cutoff
        if (outtakeSubsystem.turretReached(targetAngle))
        {
            if (globalTimer - turretTimer > 150)
            {
                if (armState != null) // i just did this if i don't really want to pass a state the correct would be to pass default in the state
                    outtakeSubsystem.armState(armState);
                if (wristState != null)
                    outtakeSubsystem.wristState(wristState);
                outtakeSubsystem.turretRawControl(0);
            }
            else
            {
                outtakeSubsystem.turretSpinTo(targetAngle);
            }
        }
        else
        {
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPIN);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPIN);
            outtakeSubsystem.turretSpinTo(targetAngle); // we
            globalTimer = turretTimer;
        }
    }
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
