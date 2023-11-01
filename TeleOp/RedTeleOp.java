package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RedTeleOp extends OpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    public void MoveDriveTrain() {
        double Vertical;
        double Horizontal;
        double Pivot;

        Vertical = gamepad1.left_stick_x;
        Horizontal = gamepad1.left_stick_y;
        Pivot = gamepad1.right_stick_x;

        FR.setPower(-Pivot + (Vertical - Horizontal));
        BR.setPower(Pivot + (Vertical + Horizontal));
        FL.setPower(-Pivot + (-Vertical + Horizontal));
        BL.setPower(Pivot + (-Vertical - Horizontal));

    }

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        MoveDriveTrain();

    }
}