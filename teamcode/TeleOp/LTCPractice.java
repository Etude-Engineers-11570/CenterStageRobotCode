package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class LTCPractice extends OpMode {
    /*DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor rightFront;
    DcMotor Lift; */
    DcMotor Intake;
    double MAX_SPEED_IN = -0.75;
    double MAX_SPEED_OUT = 0.80;
    double ZERO_SPEED = 0;
   /* public void MoveDriveTrain() {
        double Vertical;
        double Horizontal;
        double Pivot;

        Vertical = gamepad1.left_stick_x;
        Horizontal = gamepad1.left_stick_y;
        Pivot = gamepad1.right_stick_x;

        rightFront.setPower(-Pivot + (Vertical - Horizontal) / 1.75);
        rightBack.setPower(Pivot + (Vertical + Horizontal) / 1.75);
        leftFront.setPower(-Pivot + (-Vertical + Horizontal) / 1.75);
        rightFront.setPower(Pivot + (-Vertical - Horizontal) / 1.75);

    }

  */  @Override
    public void init() {

      /*  leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        Lift = hardwareMap.get(DcMotor.class, "Lift"); */
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        /* leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */




    }

    @Override
    public void loop() {
       /* MoveDriveTrain();
        Lift.setPower(gamepad2.left_stick_y * 0.75); */
        if (gamepad2.right_stick_y > 0.025) {
            Intake.setPower(MAX_SPEED_IN);
        }
        else if (gamepad2.right_stick_y < -0.025){
            Intake.setPower(MAX_SPEED_OUT);
        }
        else {
            Intake.setPower(ZERO_SPEED);
        }
     /*   if (gamepad2.left_stick_y > 0.025 || gamepad2.left_stick_y < -0.025) {
            Lift.setPower(gamepad2.left_stick_y);
        }
        else {
            Lift.setPower(ZERO_SPEED);
        } */


    }
}