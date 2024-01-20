package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.CenterStage_Detection_Pipeline_1;
import org.firstinspires.ftc.teamcode.TeleOp.LiftPID;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpHardware;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

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

public class RedAuto_Left extends LinearOpMode {

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
    CenterStage_Detection_Pipeline_1_Left pipeline;
    CenterStage_Detection_Pipeline_1_Left.PropPosition snapshotAnalysis = CenterStage_Detection_Pipeline_1_Left.PropPosition.LEFT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take


    //In the middle of the pieces
    //Pose2d poseEstimate;
    // Pose2d startingPosition = new Pose2d(12.5, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -57.5, 0));
            AutoHardware robot = new AutoHardware();
            robot.init(hardwareMap);

            //Detection Setup:
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new CenterStage_Detection_Pipeline_1_Left();
            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                // Pose2d startPose = new Pose2d(-36, 64, 0);

                Pose2d poseEstimate;
                Pose2d startingPosition = new Pose2d(12.5, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in



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
                    //This is actually right code

                    robot.Claw.setPosition(robot.ClawTurnGrab);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-36, -60, -1.57))
                                    .setTangent(0)
                                    .waitSeconds(2)
                                    .strafeToConstantHeading(new Vector2d(-36, -41)) //Go to Spike Mark Location
                                    .turn(1.70)
                                    .strafeToConstantHeading(new Vector2d(-37.5,-41))
                                    .build());
                    robot.Claw.setPosition(robot.ClawTurnStart);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-37.5, -41, 0.13))
                                    .strafeToConstantHeading(new Vector2d(-36,-41))
                                    .turn(-1.70)
                                    .strafeToConstantHeading(new Vector2d(-36, -57.5)) //Go back to starting position
                                    .turn(1.70)
                                    .strafeToConstantHeading(new Vector2d(38, -57.5)) // Move to backstage
                                    .strafeToConstantHeading(new Vector2d(57, -42)) //Go to Center Position on Backdrop
                                    .waitSeconds(1)
                                    .build());
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
                    sleep(2000);
                    robot.Dropper.setPosition(Open);
                    sleep(1000);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    sleep(2000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),0,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),0,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 0);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                        telemetry.addData("Lift2 target Height:", 0);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(-1);
                        robot.Lift2.setPower(-1);
                    }

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(57, -42, 0.13))
                                    .strafeToConstantHeading(new Vector2d(56, -42))
                                    .strafeToConstantHeading(new Vector2d(56, -20)) // go park in backstage
                                    .build());
                    break;




                }
                case CENTER: {
                    robot.Claw.setPosition(robot.ClawTurnGrab);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-36, -60, -1.57))
                                    .setTangent(0)
                                    .waitSeconds(2)
                                    .strafeToConstantHeading(new Vector2d(-36, -38)) //Go to Spike Mark Location
                                    .strafeToConstantHeading(new Vector2d(-34, -38))
                                    .build());
                    robot.Claw.setPosition(robot.ClawTurnStart);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-34, -38, -1.57))
                                    .strafeToConstantHeading(new Vector2d(-34, -57.5)) //Go back to starting position
                                    .turn(-1.70)
                                    .strafeToConstantHeading(new Vector2d(38, -57.5)) // Move to backstage
                                    .strafeToConstantHeading(new Vector2d(57, -40))//Go to Center Position on Backdrop
                                    .waitSeconds(1)
                                    .build());
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
                    sleep(2000);
                    robot.Dropper.setPosition(Open);
                    sleep(1000);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    sleep(2000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),0,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),0,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 0);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                        telemetry.addData("Lift2 target Height:", 0);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(-1);
                        robot.Lift2.setPower(-1);
                    }
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(57, -40, -3.16))
                                    .strafeToConstantHeading(new Vector2d(56, -40))
                                    .strafeToConstantHeading(new Vector2d(56, -20)) // go park in backstage
                                    .build());
                    break;

                }
                case RIGHT: {
                    robot.Claw.setPosition(robot.ClawTurnGrab);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-36, -60, -1.57))
                                    .setTangent(0)
                                    .waitSeconds(2)
                                    .strafeToConstantHeading(new Vector2d(-36, -34)) //Go to Spike Mark Location
                                    .turn(-1.70)
                                    .strafeToConstantHeading(new Vector2d(-33.5,-34))
                                    .build());
                    robot.Claw.setPosition(robot.ClawTurnStart);
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-33.5, -34, -3.27))
                                    .strafeToConstantHeading(new Vector2d(-36,-34))
                                    .strafeToConstantHeading(new Vector2d(-36, -65)) //Go back to starting position
                                    .strafeToConstantHeading(new Vector2d(38, -65)) // Move to backstage
                                    .strafeToConstantHeading(new Vector2d(57, -38)) //Go to Center Position on Backdrop
                                    .waitSeconds(1)
                                    .build());
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
                    sleep(2000);
                    robot.Dropper.setPosition(Open);
                    sleep(1000);
                    robot.Dropper_Turn1.setPosition(Straight1);
                    robot.Dropper_Turn2.setPosition(Straight2);
                    sleep(2000);
                    robot.Dropper.setPosition(Closed);

                    if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        robot.Lift1.setPower(0);
                        robot.Lift2.setPower(0);
                        if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),0,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),0,0.2)>0) {
                            robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                            robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        }
                        Lifttimer.reset();
                        telemetry.addData("Lift1 target Height:", 0);
                        telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 0, 0.2));
                        telemetry.addData("Lift2 target Height:", 0);
                        telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 0, 0.2));
                        telemetry.update();
                    }
                    else if (robot.Lift1.getCurrentPosition()>0 && robot.Lift2.getCurrentPosition()>0) { //Runs the lift to the right hight
                        telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
                        telemetry.update();
                        robot.Lift1.setPower(-1);
                        robot.Lift2.setPower(-1);
                    }
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(57, -38, -3.27))
                                    .strafeToConstantHeading(new Vector2d(56, -38))
                                    .strafeToConstantHeading(new Vector2d(56, -20)) // go park in backstage
                                    .build());
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