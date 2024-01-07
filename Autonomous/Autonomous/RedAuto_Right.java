package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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

public class RedAuto_Right extends LinearOpMode {

    public Servo Dropper = null;
    public Servo Dropper_Turn1 = null;
    public Servo Dropper_Turn2 = null;

    double Closed = 0;
    double Open = 0.1;
    double Straight1 = 0.2;
    double Straight2 = 0.2;
    double Turn1 = 0.533333333333333333;
    double Turn2 = 0.533333333333333333;
    public DcMotor Lift1;
    public DcMotor Lift2;

    public DcMotor Intake = null;

    double LiftTargetHieght = 0;
    ElapsedTime Lifttimer = new ElapsedTime();
    LiftPID CustomLiftPID = new  LiftPID(.0005,0,0,0,0);

    public TouchSensor Lift_Sensor;

    boolean isPressed;
    boolean NotPressed;



    //Webcam and detection initialization:
    OpenCvWebcam webcam;
    CenterStage_Detection_Pipeline_1 pipeline;
    CenterStage_Detection_Pipeline_1.PropPosition snapshotAnalysis = CenterStage_Detection_Pipeline_1.PropPosition.LEFT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take


    //In the middle of the pieces
    //Pose2d poseEstimate;
    // Pose2d startingPosition = new Pose2d(12.5, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -57.5, 0));
            Lift_Sensor = hardwareMap.get(TouchSensor.class, "Lift_Sensor");


            //Detection Setup:
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new CenterStage_Detection_Pipeline_1();
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
                       /* Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .setTangent(0)
                                        .strafeToConstantHeading(new Vector2d(-36, 34.5))
                                        .waitSeconds(1)
                                        .turn(3.15)
                                        .waitSeconds(1)
                                        .strafeToConstantHeading(new Vector2d(50, 34.5))
                                        .waitSeconds(1)
                                        .strafeToConstantHeading(new Vector2d(-58, 34.5))
                                        .waitSeconds(1)
                                        .strafeToConstantHeading(new Vector2d(50, 34.5))
                                        .waitSeconds(1)
                                        .strafeToConstantHeading(new Vector2d(50, 57.5))
                                        .waitSeconds(1)
                                        .build()); */

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
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(12.5, -60, 1.57))
                                    .setTangent(0)
                                    .strafeToConstantHeading(new Vector2d(12.5, -34.5)) //Go to Spike Mark Location
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(4,-34.5 )) // go to score on Left Spikemark
                                    .waitSeconds(1)
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(4, -34.5, -1.57))
                                    .strafeToConstantHeading(new Vector2d(12.5, -50)) //Go back to starting position
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(38, -50)) // Move to backstage
                                    .waitSeconds(1)
                                    .turn(3.15 / 2) //Turn to face backdrop
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(50, -34.5)) //Go to Center Position on Backdrop
                                    .waitSeconds(1)
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(50, -34.5, 3.15))
                                    .strafeToConstantHeading(new Vector2d(50, -57.5)) // go park in backstage
                                    .waitSeconds(1)
                                    .build());
                }
                case CENTER: {
                   Actions.runBlocking(
                           drive.actionBuilder(new Pose2d(12.5, -60, -1.57)) // Changed y-coordinate from -60 to -50 for test
                                   .setTangent(0)
                                   .strafeToConstantHeading(new Vector2d(12.5, -32)) //Go to Place on Center Spike Mark | Changed from -32 to -20 for test
                                   .waitSeconds(1)
                                   .build());
                   Actions.runBlocking(
                           drive.actionBuilder(new Pose2d(12.5, -32, -1.57)) //Changed y-coordinate from -32 to -20 for test
                                   .strafeToConstantHeading(new Vector2d(12.5, -50)) //Go back to starting position | Changed from -50 to -40 for test
                                   .waitSeconds(1)
                                   .strafeToConstantHeading(new Vector2d(38, -50)) // Move to backstage | changed from -50 to -40 for test
                                   .waitSeconds(1)
                                   .turn(3.15 / 2) //Turn to face backdrop
                                   .waitSeconds(1)
                                   .strafeToConstantHeading(new Vector2d(50, -34.5)) //Go to Center Position on Backdrop | changed to -24.5 from -34.5 for test
                                   .waitSeconds(1)
                                   .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(50,-34.5,3.15)) // changed from -34.5 to -24.5 for test
                                    .strafeToConstantHeading(new Vector2d(50, -57.5)) // go park in backstage // changed from -57.5 to -47.5 for test
                                    .waitSeconds(1)
                                    .build());
                }
                case RIGHT: {
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(12.5, -60, -1.57))
                                    .setTangent(0)
                                    .strafeToConstantHeading(new Vector2d(12.5, -34.5)) //Go to Spike Mark Location
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(20,-34.5 )) // go to score on Left Spikemark
                                    .waitSeconds(1)
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(4, -34.5, -1.57))
                                    .strafeToConstantHeading(new Vector2d(12.5, -50)) //Go back to starting position
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(38, -50)) // Move to backstage
                                    .waitSeconds(1)
                                    .turn(3.15 / 2) //Turn to face backdrop
                                    .waitSeconds(1)
                                    .strafeToConstantHeading(new Vector2d(50, -34.5)) //Go to Center Position on Backdrop
                                    .waitSeconds(1)
                                    .build());
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(50, -34.5, 3.15))
                                    .strafeToConstantHeading(new Vector2d(50, -57.5)) // go park in backstage
                                    .waitSeconds(1)
                                    .build());

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