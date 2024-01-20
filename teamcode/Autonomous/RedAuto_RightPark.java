package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class RedAuto_RightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12.5, -50, -3.14));

            waitForStart();
           /* Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-36,-64, 3.14))
                            .waitSeconds(5)
                            .strafeToConstantHeading(new Vector2d(12.5, -34.5))
                            .turn( -1.57)
                            .strafeToConstantHeading(new Vector2d(60, -34.5))
                            .build()); */
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(12.5, -50, -3.14))
                            .waitSeconds(1)
                            .lineToYConstantHeading( -9)//Go to Spike Mark Location
                            .build());
        }
    }}
