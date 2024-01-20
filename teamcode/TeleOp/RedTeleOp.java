// /Copyright (c) 2017 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;


//import org.firstinspires.ftc.teamcode.control.Mecanum;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RedTeleOp", group="Pushbot")
//@Disabled
public class RedTeleOp extends LinearOpMode {

    public RedTeleOp() {
    }

    public void MoveDriveTrain() {
        double Vertical;
        double Horizontal;
        double Pivot;

        Vertical = -gamepad1.left_stick_x;
        Horizontal = gamepad1.left_stick_y;
        Pivot = gamepad1.right_stick_x;

       // robot.FR.setPower(-Pivot + (Vertical - Horizontal));
       // robot.BR.setPower(Pivot + (Vertical + Horizontal));
       // robot.FL.setPower(-Pivot + (-Vertical + Horizontal));
       // robot.BL.setPower(Pivot + (-Vertical - Horizontal));

        robot.leftFront.setPower(-Pivot + (-Vertical + Horizontal));
        robot.leftBack.setPower(Pivot + (-Vertical - Horizontal));
        robot.rightBack.setPower(Pivot + (Vertical + Horizontal));
        robot.rightFront.setPower(-Pivot + (Vertical - Horizontal));

    }

    /* Declare OpMode members. */
    TeleOpHardware robot = new TeleOpHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    double LiftTargetHieght = 0;
    ElapsedTime Lifttimer = new ElapsedTime();
    LiftPID CustomLiftPID = new  LiftPID(.0005,0,0,0,0);

    static final double COUNTS_PER_MOTOR_REV = 4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 1.6875;     // For figuring circumference

    double liftspeed = 1;
   // double Multiplyer = 3;
   // double DrivePower = 1.0;
    double IntakeSpeed = 1;

    public Encoder par0;

    public Encoder par1;
    public  Encoder perp;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;


    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static ThreeDeadWheelLocalizer.Params PARAMS = new ThreeDeadWheelLocalizer.Params();

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
       // robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       /* robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */


       /* par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));

        par0 = hardwareMap.get(Encoder.class, "leftFront");
        par1 = hardwareMap.get(Encoder.class, "rightFront");
        perp = hardwareMap.get(Encoder.class, "rightBack");

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position; */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

       // RevBlinkinLedDriver revBlinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class,"LED1");
       // RevBlinkinLedDriver revBlinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "LED2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MoveDriveTrain();
            //Always running:
            //DrivePower = 1 / ( 1 + gamepad1.right_trigger * Multiplyer ); //lowest speed is 1/1+multiplyer, rn 1/4
          /*  revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI); */



            if(gamepad2.left_stick_y>0)
            {
                robot.Lift1.setPower(0); //When we are all the way down we the intake automatically
                LiftTargetHieght = robot.Lift1.getCurrentPosition();
                robot.Lift2.setPower(0);
                LiftTargetHieght = robot.Lift2.getCurrentPosition();
            }
            if (gamepad2.left_stick_y > 0.025 || gamepad2.left_stick_y < -0.025) {
                robot.Lift1.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                LiftTargetHieght = robot.Lift1.getCurrentPosition();
                robot.Lift2.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                LiftTargetHieght = robot.Lift2.getCurrentPosition();
                telemetry.addData("Lifting", -gamepad2.left_stick_y);
                telemetry.update();

            }

            else
            {

                robot.Lift1.setPower(0); //Stopped Lift
                if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                    robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), LiftTargetHieght, 0.02));
                }
                if(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                    robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), LiftTargetHieght, 0.02));
                }
                /*  if (LiftTargetHieght > 16000) {
                robot.Lift1.setPower(0);
                robot.Lift2.setPower(0);
            } */

          /*  if (gamepad2.left_stick_y > 0.025 || gamepad2.left_stick_y < -0.025) {
                robot.Lift1.setPower(gamepad2.left_stick_y * liftspeed); //Lifting
                robot.Lift1.setPower(gamepad2.left_stick_y * liftspeed); //Lifting
            }
            else {
                robot.Lift1.setPower(0);
                robot.Lift1.setPower(0);
            } */
          /*  telemetry.addData("Lift target Height:", LiftTargetHieght);
            telemetry.addData("Lift Current Height:", robot.Lift1.getCurrentPosition());
            telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), LiftTargetHieght, 0.02));

            telemetry.addData("Lift target Height:", LiftTargetHieght);
            telemetry.addData("Lift Current Height:", robot.Lift2.getCurrentPosition());
            telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), LiftTargetHieght, 0.02));
*/
           // robot.Lift1.setPower(-gamepad2.left_stick_y);
           // robot.Lift2.setPower(-gamepad2.left_stick_y);



                //OutTake
            if (gamepad2.left_bumper) {
                robot.Dropper.setPosition(robot.Open);
            }
            else {
                robot.Dropper.setPosition(robot.Closed);
            }
           /* if (gamepad2.dpad_up) {
                robot.Dropper_Turn1.setPosition(gamepad2.left_stick_x / 2);
                robot.Dropper_Turn2.setPosition(gamepad2.left_stick_x / 2);
            } */

            //Intake
            if (gamepad2.right_stick_y > 0.025 || gamepad2.right_stick_y < 0.025) {
            robot.Intake.setPower(gamepad2.right_stick_y * IntakeSpeed); }
            else {
                robot.Intake.setPower(0);
            }
            if (gamepad2.right_bumper) {
                robot.Dropper_Turn1.setPosition(robot.Turn1);
                robot.Dropper_Turn2.setPosition(robot.Turn2);
            }
            else
            {
                robot.Dropper_Turn1.setPosition(robot.Straight1);
                robot.Dropper_Turn2.setPosition(robot.Straight2);
            }
            if (gamepad2.left_trigger > 0.025) {
                robot.Hook.setPower(gamepad2.left_trigger);
            }
            else if (gamepad2.right_trigger > 0.025) {
                robot.Hook.setPower(-gamepad2.right_trigger);
            }
            else {
                robot.Hook.setPower(0);
            }

            telemetry.addData("perp: ", lastPerpPos);
            telemetry.addData("par1:", lastPar1Pos);
            telemetry.addData("par0:", lastPar0Pos);
            telemetry.update();

            if (gamepad1.left_bumper) {
                robot.Launcher.setPosition(robot.LAUNCH);
            }
            else {
                robot.Launcher.setPosition(robot.NO_LAUNCH);
            }

               /* telemetry.addData("Red1", robot.OutTakeDetector1.red());
                telemetry.addData("Green1", robot.OutTakeDetector1.green());
                telemetry.addData("Blue1", robot.OutTakeDetector1.blue());
                telemetry.addData("Red2", robot.OutTakeDetector2.red());
                telemetry.addData("Green2", robot.OutTakeDetector2.green());
                telemetry.addData("Blue2", robot.OutTakeDetector2.blue());
                telemetry.update(); */

             /*   boolean isWhiteLeft = robot.OutTakeDetector1.red() > 200 && robot.OutTakeDetector1.blue() > 200 && robot.OutTakeDetector1.green() > 200;
                boolean isYellowLeft = robot.OutTakeDetector1.red() > 200 && robot.OutTakeDetector1.blue() > 200 && robot.OutTakeDetector1.green() < 50;
                boolean isGreenLeft = robot.OutTakeDetector1.red() < 50 && robot.OutTakeDetector1.blue() < 50 && robot.OutTakeDetector1.green() > 200;
                boolean isNothingLeft = robot.OutTakeDetector1.red() > 200 && robot.OutTakeDetector1.blue() < 50 && robot.OutTakeDetector1.green() < 50;
                boolean isPurpleLeft = robot.OutTakeDetector1.red() > 200 && robot.OutTakeDetector1.blue() < 50 && robot.OutTakeDetector1.green() < 50;

                boolean isWhiteRight = robot.OutTakeDetector2.red() > 200 && robot.OutTakeDetector2.blue() > 200 && robot.OutTakeDetector2.green() > 200;
                boolean isYellowRight = robot.OutTakeDetector2.red() > 200 && robot.OutTakeDetector2.blue() > 200 && robot.OutTakeDetector2.green() < 50;
                boolean isGreenRight = robot.OutTakeDetector2.red() < 50 && robot.OutTakeDetector2.blue() < 50 && robot.OutTakeDetector2.green() > 200;
                boolean isNothingRight = robot.OutTakeDetector2.red() > 200 && robot.OutTakeDetector2.blue() < 50 && robot.OutTakeDetector2.green() < 50;
                boolean isPurpleRight = robot.OutTakeDetector2.red() > 150 && robot.OutTakeDetector2.blue() > 150 && robot.OutTakeDetector2.green() < 100;


                    if (isWhiteLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);



                    }
                    if (isWhiteRight) {
                        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);


                    }
                    if (isYellowLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);


                    }
                    if (isYellowRight) {
                        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);


                    }
                    if (isGreenLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


                    }
                    if (isGreenRight) {
                        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


                    }
                    if (isPurpleLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


                    }
                    if (isPurpleRight) {
                        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


                    }
                   if (isNothingLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);



                    }
                    if (isNothingRight) {
                        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);


                    } */
            }
    }
}}

