package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideAutoFar;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.accessory.SideAfterAuto;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Autonomous(name = "5+0 Far", group = "Far")
public class SpecimenAuto extends LinearOpMode
{

    enum autoState {
        PRELOAD_DEPOSIT,
        AFTER_SUB_INTAKE,
        EJECTION_TO_HP,
        SAMPLE_PICKUP,
        INTAKE,
        DEPOSIT_DRIVE,
        DROP,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PathsFarExtra trajectories = new PathsFarExtra();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    GeneralHardware.Side side = GeneralHardware.Side.Red;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;
    int pickupCycle = 0;
    boolean intakedSpec = false;
    boolean attemptedIntake = false;
    boolean startedExtending = false;
    double xPosition, yPosition, headingPosition;
    double headingErrorToEndPose;
    boolean didWeOpenedTheStupidFuckingClaw = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY;
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
            if (delay(1000))
            {
                intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
            }
            if (delay(1500))
            {
                outtakeSubsystem.armSetPos(0.25);
            }
            if (gamepad2.dpad_up || gamepad1.dpad_up) side = GeneralHardware.Side.Red;
            else if (gamepad2.dpad_down || gamepad1.dpad_down) side = GeneralHardware.Side.Blue;

            if (side == GeneralHardware.Side.Blue) gamepad2.setLedColor(0, 0, 255, 1000);
            else gamepad2.setLedColor(255, 0, 0, 1000);

            SideAfterAuto.side = side;
            telemetry.addData("Side", side);
            telemetry.update();
            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.PRELOAD_DEPOSIT || state == autoState.EJECTION_TO_HP || state == autoState.SAMPLE_PICKUP); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true, "black");
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Cycle", cycle);
//            telemetry.addData("IntakeSlide Position", intakeSubsystem.slidePosition);
//            telemetry.addData("Intake Slides", hardware.intakeSlidesM.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Lift slides", hardware.outtakeLiftM.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Intaked spec", intakedSpec);
//            telemetry.addData("Reached third intake", trajectories.thirdIntake.isFinished());
//            telemetry.addData("Pickup cycle", pickupCycle);
//            telemetry.addData("Is red", intakeSubsystem.isRed);
//            telemetry.addData("Is slide over 14in", intakeSubsystem.slideOverPosition(14));
//            telemetry.addData("HeadingPosition", Math.toDegrees(headingPosition));
//            telemetry.addData("Heading Error To End Of Trajectory", headingErrorToEndPose);
//            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.update();
        }
        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                if (trajectories.preloadTrajectory.isFinished())
                {
                    state = autoState.DROP;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    cycle ++;
                    attemptedIntake = false;
                    resetTimer();
                    break;
                }
                if (delay(80))
                    hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                if (delay(0))
                {
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);
                    outtakeSubsystem.liftToInternalPIDTicks(730);
                    //outtakeLiftPresets(false, false);
                }
                break;
            case EJECTION_TO_HP:
                Trajectory ejectionTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                        ejectionTrajectory = trajectories.subToFirstE;
                        break;
                    case 1:
                        ejectionTrajectory = trajectories.secondE;
                        break;
                    case 2:
                        ejectionTrajectory = trajectories.thirdE;
                        break;
                    case 3:
                        ejectionTrajectory = trajectories.forthE; // this path is horrible
                        break;
                }
                if (ejectionTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(ejectionTrajectory);
                    if (delay(40))
                    {
                        headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - ejectionTrajectory.getFinalPose().getHeading()));
                        boolean reachedFinalHeading =  headingErrorToEndPose < 2;

                        intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                        if (
                        (Math.toDegrees(headingPosition) < -50)
                        )
                        {
                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        }
//                        else if (ejectionTrajectory.isFinished() && reachedFinalHeading && intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) > 16)
//                        {
//                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
//                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
//                        }

                        if ((pickupCycle == 0 && headingErrorToEndPose < 10) || headingErrorToEndPose < 15)
                        {
                            switch (pickupCycle) // leave as a switch until i know everything works
                            {
                                case 0:
                                    intakeSubsystem.intakeSlideInternalPID(18.5);
                                    break;
                                case 1:
                                    intakeSubsystem.intakeSlideInternalPID(16);
                                    break;
                                case 2:
                                    intakeSubsystem.intakeSlideInternalPID(20.5);
                                    break;
                                case 3:
                                    intakeSubsystem.intakeSlideInternalPID(18.5);
                                    break;
                            }
                        }
                        else if (pickupCycle == 0) intakeSubsystem.intakeSlideInternalPID(0);
                        //else if (pickupCycle != 3) intakeSubsystem.intakeSlideInternalPID(5);
                        //if (delay(600) && ejectionTrajectory.isFinished() && reachedFinalHeading && intakeSubsystem.getColorValue() < 100)
                        if (delay(600) && intakeSubsystem.getColorValue() < 100)
                        {
                            state = pickupCycle == 3 ? autoState.INTAKE : autoState.SAMPLE_PICKUP;
                            if (pickupCycle == 3) intakeClipHoldLogic(0, 8);

                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                            resetTimer();
                            break;
                        }
                    }
                }
                break;
            case SAMPLE_PICKUP:
                Trajectory pickupTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                        pickupTrajectory = trajectories.SPFromSub;
                        break;
                    case 1:
                        pickupTrajectory = trajectories.secondSP;
                        break;
                    case 2:
                        pickupTrajectory = trajectories.thirdSP;
                        break;
                }
                if (pickupTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(pickupTrajectory);
                    if (delay(40))
                    {
                        if (pickupCycle == 0 && delay(450)) // this makes sure we reset after the preload
                        {
                            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                            outtakeSubsystem.liftToInternalPIDTicks(0);
                        }

                        headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - pickupTrajectory.getFinalPose().getHeading()));
                        boolean reachedFinalHeading =  headingErrorToEndPose < 2;

                        if (headingErrorToEndPose < 12)
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                        }
                        if (delay(140))
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        }


                        if (reachedFinalHeading || startedExtending)//(headingPosition > (pickupCycle == 0 ? Math.toRadians(25) : Math.toRadians(15)))
                        {
                            switch (pickupCycle)
                            {
                                case 0:
                                    intakeSubsystem.intakeSlideInternalPID(13);
                                    break;
                                case 1:
                                    intakeSubsystem.intakeSlideInternalPID(10);
                                    break;
                                case 2:
                                    intakeSubsystem.intakeSlideInternalPID(15.5);
                                    break;
                            }
                            startedExtending = true;
                        }
                        else if (pickupCycle == 0) intakeSubsystem.intakeSlideInternalPID(2);
                        else if (pickupCycle == 1) intakeSubsystem.intakeSlideInternalPID(6);
                        else if (pickupCycle == 2) intakeSubsystem.intakeSlideInternalPID(10);

                        if ((delay(300) && intakeSubsystem.getColorValue() > 500 && pickupTrajectory.isFinished())
                                ||
                                (   (pickupCycle == 0 && delay(2300)) ||
                                    (pickupCycle == 1 && delay(1700)) ||
                                    (pickupCycle == 2 && delay(1700))
                                )
                        )
                        {
                            state = autoState.EJECTION_TO_HP;
                            startedExtending = false;
                            pickupCycle++;
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                            resetTimer();
                            break;
                        }
                    }
                }
                break;
            case INTAKE:
                if (intakedSpec && delay(190) && hardware.drive.stopped())
                {
                    state =  autoState.DEPOSIT_DRIVE;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(0, 10);
                if (!intakedSpec)
                {

                    if (cycle == 2)
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    }
                    switch (cycle)
                    {
                        case 2: // this case has to be a little different
                            headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - trajectories.firstIntakePart1.getFinalPose().getHeading()));
                            if (headingErrorToEndPose < 2 && trajectories.firstIntakePart1.isFinished())
                                hardware.drive.followTrajectorySplineHeading(trajectories.firstIntakePart2);
                            else
                                hardware.drive.followTrajectorySplineHeading(trajectories.firstIntakePart1);
                            if (trajectories.firstIntakePart2.isFinished() && hardware.drive.stopped())
                            {
                                //outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                outtakeSubsystem.clawSetPos(0.5);
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 3:
                            hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                            if (trajectories.secondIntake.isFinished() && hardware.drive.stopped())
                            {
                                //outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                outtakeSubsystem.clawSetPos(0.5);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 4:
                            hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                            if (trajectories.thirdIntake.isFinished() && hardware.drive.stopped())
                            {
                                //outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                outtakeSubsystem.clawSetPos(0.5);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 5:
                            hardware.drive.followTrajectorySplineHeading(trajectories.forthIntake);
                            if (trajectories.forthIntake.isFinished() && hardware.drive.stopped())
                            {
                                //outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                outtakeSubsystem.clawSetPos(0.5);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 6:
                            hardware.drive.followTrajectorySplineHeading(trajectories.fifthIntake);
                            if (trajectories.fifthIntake.isFinished() && hardware.drive.stopped())
                            {
//                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                outtakeSubsystem.clawSetPos(0.5);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                    }

                    if (delay(40))
                    {
                        if (delay(140))
                        {
                            outtakeSubsystem.liftToInternalPIDTicks(0);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            if (!intakedSpec)
                            {
                                outtakeSubsystem.clawSetPos(0.85); // open
                            }
                        }
                        if (delay(300))
                        {
                            //outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                            outtakeSubsystem.railSetPos(0.57);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        }

                        //if (delay(550))
                        //intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    }
                }
                else
                {
                    outtakeSubsystem.clawSetPos(0.5);
                    if (delay(90))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                        outtakeSubsystem.liftToInternalPIDTicks(730);
                        if (delay(120))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                        }
                        if (cycle == 2)
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                        }
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                if (( // THIS ONLY RUNS ON CYCLE 1 ONWARDS
                        (cycle == 2 && trajectories.firstDeposit.isFinished()) ||
                        (cycle == 3 && trajectories.secondDeposit.isFinished()) ||
                        (cycle == 4 && trajectories.thirdDeposit.isFinished()) ||
                        (cycle == 5 && trajectories.forthDeposit.isFinished()) ||
                        (cycle == 6 && trajectories.fifthDeposit.isFinished()))
                )
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(0, 8);

                switch (cycle)
                {
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstDeposit);
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondDeposit);
                        break;
                    case 4:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdDeposit);
                        break;
                    case 5:
                        hardware.drive.followTrajectorySplineHeading(trajectories.forthDeposit);
                        break;
                    case 6:
                        hardware.drive.followTrajectorySplineHeading(trajectories.fifthDeposit);
                        break;
                }
                if (delay(20))
                {
                    outtakeSubsystem.liftToInternalPIDTicks(800);
                    outtakeSubsystem.wristSetPos(0.42);

                }
//                if (delay(200))
//                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
//                else if (delay(20))
//                {
//                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
//                }

                if (delay(100))
                {
                    //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);

                    if (delay(200))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                    if (delay(300))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                    }
                }
                break;
            case DROP:
                if (delay(200))//(cycle == 0 ? delay(400) : delay(230))
                {
                    state = cycle == 1 ? autoState.SAMPLE_PICKUP : cycle == 5 ? autoState.PARK : autoState.INTAKE;
                    cycle++;
                    intakedSpec = false;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.wristSetPos(0.47);
                if (delay(70))
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (cycle == 0 && delay(90))
                {
                    intakeSubsystem.intakeSlideInternalPID(0);
                }
                if (delay(120))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                }
                break;
            case PARK:
                if (trajectories.parkTrajectory.isFinished() && delay(400))
                {
                    state = autoState.IDLE;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(0, 8);
                if (delay(400))
                {
                    outtakeSubsystem.liftToInternalPIDTicks(0);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.MIDDLE);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.parkTrajectory);
                break;
            case IDLE: // we idle here duuhhh
                break;
        }

    }
    public void outtakeLiftPresets(boolean isSample, boolean low)
    {
        if (isSample)
        {
            if (low) outtakeSubsystem.liftToInternalPIDTicks(350);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(1655);
            //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
        }
        else
        {
            if (low)  outtakeSubsystem.liftToInternalPIDTicks(0);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(700);
            //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
        }
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

    public boolean delay(double delayTime)
    {
        return (globalTimer - sequenceTimer) > delayTime;
    }
    public void resetTimer()
    {
        sequenceTimer = globalTimer;
    }
}
