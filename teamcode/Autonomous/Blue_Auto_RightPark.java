package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class Blue_Auto_RightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -57.5, 1.57));

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-36,64, 1.57))
                            .waitSeconds(5)
                            .strafeToConstantHeading(new Vector2d(12.5, 34.5))
                            .turn(3.15 / 2)
                            .strafeToConstantHeading(new Vector2d(50, 34.5))
                            .build());



        }
    }}
