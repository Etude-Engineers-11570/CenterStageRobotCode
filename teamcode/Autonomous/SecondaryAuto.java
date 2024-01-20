package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Notification;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.LiftPID;

@Autonomous
public class SecondaryAuto extends LinearOpMode {
    double LiftTargetHieght = 0;
    ElapsedTime Lifttimer = new ElapsedTime();
    LiftPID CustomLiftPID = new  LiftPID(.003,0,0,0,0);

    double Closed = 0;
    double Open = 0.1;
    double Straight1 = 0.2;
    double Straight2 = 0.2;
    double Turn1 = 0.533333333333333333;
    double Turn2 = 0.533333333333333333;

    double HighLift = 280;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoHardware robot = new AutoHardware();
        robot.init(hardwareMap);

        waitForStart();

        if (robot.Lift1.getCurrentPosition()<280 && robot.Lift2.getCurrentPosition()<280) { //Runs the lift to the right hight
            robot.Lift1.setPower(0);
            robot.Lift2.setPower(0);
            if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),280,0.2)>0 && CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),280,0.2)>0) {
                robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 280, 0.2));
                robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 280, 0.2));
            }
            Lifttimer.reset();
            telemetry.addData("Lift1 target Height:", 280);
            telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
            telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 280, 0.2));
            telemetry.addData("Lift2 target Height:", 280);
            telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
            telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 280, 0.2));
            telemetry.update();
        }
        else if (robot.Lift1.getCurrentPosition()<280 && robot.Lift2.getCurrentPosition()<280) { //Runs the lift to the right hight
            telemetry.addData("Height:", robot.Lift1.getCurrentPosition());
            telemetry.update();
            robot.Lift1.setPower(1);
            robot.Lift2.setPower(1);
        }
    }}
