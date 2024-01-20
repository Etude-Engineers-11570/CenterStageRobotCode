package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {

    public Servo Dropper = null;
    public Servo Dropper_Turn1 = null;
    public Servo Dropper_Turn2 = null;
    public Servo Claw = null;

    double Closed = 0;
    double Open = 0.1;
    double Straight = 0;
    double Turn1 = 0.166666666667;
    double Turn2 = -0.166666666667;

    double ClawTurnGrab = 0.0645;
    double ClawTurnStart = 0.01;


    @Override
    public void runOpMode() throws InterruptedException {
        Dropper = hardwareMap.get(Servo.class, "Dropper");
        Dropper_Turn1 = hardwareMap.get(Servo.class, "Dropper_Turn1");
        Dropper_Turn2 = hardwareMap.get(Servo.class, "Dropper_Turn2");
        Claw = hardwareMap.get(Servo.class, "Claw");

        Dropper.setPosition(Closed);
        Dropper_Turn1.setPosition(Straight);
        Dropper_Turn2.setPosition(Straight);
        Claw.setPosition(ClawTurnStart);


        waitForStart();

        while (opModeIsActive()) {
        if (gamepad2.left_bumper) {
            Dropper.setPosition(Open);
        }
        else {
            Dropper.setPosition(Closed);
        }

        if (gamepad2.a) {
            Dropper_Turn1.setPosition(Turn1);
            Dropper_Turn2.setPosition(Turn2);
        }
        if (gamepad2.b)
        {
            Dropper_Turn1.setPosition(Straight);
            Dropper_Turn2.setPosition(Straight);
        }
        if (gamepad2.dpad_up) {
            Claw.setPosition(ClawTurnGrab);
        }
        else {
            Claw.setPosition(ClawTurnStart);
        }


    }
}}
