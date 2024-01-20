package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class RedAuto_LeftPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12.5, -64, 1.57));

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(12.5,-64, 1.57))
                            .waitSeconds(5)
                            .strafeToConstantHeading(new Vector2d(12.5, -10))
                            .turn(1.57)
                            .strafeToConstantHeading(new Vector2d(100, -10))
                            .build());
        }
    }}
