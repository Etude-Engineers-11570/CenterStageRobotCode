package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.LiftPID;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */

@Autonomous(group = "drive")

public class RedAuto_Right_2_2 extends LinearOpMode {

    double LiftTargetHieght = 0;
    ElapsedTime Lifttimer = new ElapsedTime();
    LiftPID CustomLiftPID = new  LiftPID(.0005,0,0,0,0);

    double Closed = 0;
    double Open = 0.1;
    double Straight1 = 0.2;
    double Straight2 = 0.2;
    double Turn1 = 0.533333333333333333;
    double Turn2 = 0.533333333333333333;

    //Webcam and detection initialization:
    OpenCvWebcam webcam;
    SkystoneDetermination_RedRight pipeline;
    SkystoneDetermination_RedRight.SkystonePosition snapshotAnalysis = SkystoneDetermination_RedRight.SkystonePosition.RIGHT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take


    //In the middle of the pieces
    //Pose2d poseEstimate;
    // Pose2d startingPosition = new Pose2d(12.5, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15, -60, -Math.PI/2));
            AutoHardware robot = new AutoHardware();
            robot.init(hardwareMap);

            //Detection Setup:
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new SkystoneDetermination_RedRight();
            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                // Pose2d startPose = new Pose2d(-36, 64, 0);

                Pose2d poseEstimate;
                Pose2d startingPosition = new Pose2d(12.5, -60, Math.toRadians(90)); //If we Spin First, off by 1.5in



                @Override
                public void onError(int errorCode) {
                }
            });

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested()) {
                telemetry.addData("Realtime analysis", pipeline.getAnalysis());
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
                switch (pipeline.getAnalysis()) {
                    case LEFT: {
                    }
                    case CENTER: {
                    }
                    case RIGHT: {
                    }
                }

            }

            /*
             * The START command just came in: snapshot the current analysis now
             * for later use. We must do this because the analysis will continue
             * to change as the camera view changes once the robot starts moving!
             */
            snapshotAnalysis = pipeline.getAnalysis();
            switch (snapshotAnalysis) {
                case LEFT: {
                    robot.Claw.setPosition(robot.ClawTurnGrab);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(15, -60, -Math.PI/2))
                                    .waitSeconds(1)
                                    .lineToYConstantHeading(-45)
                                    .turn(Math.PI/2)
                                    .lineToXConstantHeading(14)
                                    .build()
                    );

                    robot.Claw.setPosition(robot.ClawTurnStart);


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(14, -45, 0))
                                    .lineToXConstantHeading(25)
                                    .turn(-Math.PI/2)
                                    .lineToYConstantHeading(-55)
                                    .splineToSplineHeading(new Pose2d(50, -37, Math.PI), -Math.PI/2)
                                    .build()
                    );

                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    //sleep(1000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(50, -37, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(44, -52))
                                    .splineToConstantHeading(new Vector2d(30, -57.75), -Math.PI)
                                    .lineToXConstantHeading(-25)
                                    .splineToSplineHeading(new Pose2d(-31.1, -24.8, Math.PI), Math.PI)
                                    .build());
                    robot.PixelClaw.setPosition(robot.PixelTurn);
                    sleep(1000);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-31, -25, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-27, -23))
                                    .build()
                    );
                    robot.PixelClaw.setPosition(robot.PixelStart);
                    sleep(250);
                    robot.Dropper.setPosition(Open);
                    robot.Intake.setPower(-1);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-27, -23, Math.PI))
                                    .lineToX(-29)
                                    .waitSeconds(2)
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-30, -23, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-20, 0))
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-20, -5, Math.PI))
                                    .lineToXConstantHeading(30)
                                    .splineToConstantHeading(new Vector2d(54, -40), 0)
                                    .build()
                    );
                    robot.Intake.setPower(0);
                    robot.Dropper.setPosition(Closed);
                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    //sleep(1000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(52, -40, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(48, -65))
                                    .build()
                    );
                    break;
                }
                case CENTER: {

                    robot.Claw.setPosition(robot.ClawTurnGrab);

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(15, -60, -Math.PI/2))
                                    .waitSeconds(1)
                                    .lineToYConstantHeading(-40)
                                    .strafeToConstantHeading(new Vector2d(9, -41))
                                    .build()
                    );

                    robot.Claw.setPosition(robot.ClawTurnStart);


                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(9, -41, -Math.PI/2))
                                    .strafeToConstantHeading(new Vector2d(15, -50))
                                    .splineToSplineHeading(new Pose2d(47, -40, Math.PI), -Math.PI/2)
                                    .build()
                    );

                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(47, -39, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(30, -58))
                                   // .splineToConstantHeading(new Vector2d(30, -57), Math.PI)
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(30, -57, Math.PI))
                                    .lineToXConstantHeading(-25)
                                    .splineToSplineHeading(new Pose2d(-34.5, -24, Math.PI), Math.PI)
                                    .build()
                    );
                    robot.PixelClaw.setPosition(robot.PixelTurn);
                    sleep(1000);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-34.5, -24, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-27, -23))
                                    .build()
                    );
                    robot.PixelClaw.setPosition(robot.PixelStart);
                    sleep(250);
                    robot.Dropper.setPosition(Open);
                    robot.Intake.setPower(-1);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-27, -23, Math.PI))
                                    .lineToX(-30)
                                    .waitSeconds(2)
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-30, -23, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-20, 0))
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-20, -5, Math.PI))
                                    .lineToXConstantHeading(30)
                                    .splineToConstantHeading(new Vector2d(54, -36), 0)
                                    .build()
                    );
                    robot.Intake.setPower(0);
                    robot.Dropper.setPosition(Closed);
                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    //sleep(1000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(52, -40, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(48, -57.5))
                                    .build()
                    );
                    break;
                }
                case RIGHT: {

                    robot.Claw.setPosition(robot.ClawTurnGrab);

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(15, -60, -Math.PI/2))
                                    .waitSeconds(0.75)
                                    .lineToYConstantHeading(-47.5)
                                    .build()
                    );

                    robot.Claw.setPosition(robot.ClawTurnStart);

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(15, -47.5, -Math.PI/2))
                                    .lineToYConstantHeading(-50)
                                    .splineToSplineHeading(new Pose2d(45, -42, Math.PI), -Math.PI/2)
                                    .build()

                    );

                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(50, -44, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(30, -59))
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(30, -59, Math.PI))
                                    .lineToXConstantHeading(-25)
                                    .splineToSplineHeading(new Pose2d(-32, -25.5, Math.PI), Math.PI)
                                    .build()
                    );
                    robot.PixelClaw.setPosition(robot.PixelTurn);
                    sleep(1000);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-31, -25.5, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-27, -23))
                                    .build()
                    );
                    robot.PixelClaw.setPosition(robot.PixelStart);
                    sleep(250);
                    robot.Dropper.setPosition(Open);
                    robot.Intake.setPower(-1);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-27, -23, Math.PI))
                                    .lineToX(-29.5)
                                    .waitSeconds(2)
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-30, -23, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(-17.5, 0))
                                    .build()
                    );
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-20, -5, Math.PI))
                                    .lineToXConstantHeading(30)
                                    .splineToConstantHeading(new Vector2d(54, -36), 0)
                                    .build()
                    );
                    robot.Intake.setPower(0);
                    robot.Dropper.setPosition(Closed);
                    if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),560,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),560,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 560);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
                        telemetry.addData("Lift2 target Height:", 560);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()<560 && robot.Lift2.getCurrentPosition()<560) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    robot.Dropper_Turn1.setPosition(Turn1);
                    robot.Dropper_Turn2.setPosition(Turn2);
                    sleep(500);
                    robot.Dropper.setPosition(Open);
                    sleep(1500);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    //sleep(1000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition() < (20) && robot.Lift2.getCurrentPosition() < (20)) {
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                    }
                    else {
                        robot.Lift1.setPower(-0.5);
                        robot.Lift2.setPower(-0.5);
                    }
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(54, -36, Math.PI))
                                    .strafeToConstantHeading(new Vector2d(46, -60))
                                    .build()
                    );
                    break;



                }}
            if (isStopRequested()) return;

            //Set the current state to DrivetoGamepeice, our first step, Then have it follow that trajectory
            //Because of async it lets the program continue
            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.
                // We essentially define the flow of the state machine through this switch statement
            }}}}