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

@TeleOp(name="BlueTeleOp", group="Pushbot")
//@Disabled
public class BlueTeleOp extends LinearOpMode {

    public BlueTeleOp() {
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
    double Multiplyer = 2.5;
     double DrivePower = 0.5;
    double IntakeSpeed = -1;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        RevBlinkinLedDriver revBlinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class,"LED2");
        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        RevBlinkinLedDriver revBlinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class,"LED1");
        revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("y", -gamepad1.left_stick_y);
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("z", -gamepad1.right_stick_x);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           // MoveDriveTrain();
            
            setDriveForMecanum(Mecanum.joystickToMotion(
                    -gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, -gamepad1.right_stick_y));
            //Always running:
            DrivePower = 1 / ( 1 + gamepad1.right_trigger * Multiplyer ); //lowest speed is 1/1+multiplyer, rn 1/4

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
//                telemetry.addData("Lifting", -gamepad2.left_stick_y);
//                telemetry.addData("Lift1 target Height:", 560);
//                telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
//                telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
//                telemetry.addData("Lift2 target Height:", 560);
//                telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
//                telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
//                telemetry.update();

            }

            else
            {

                robot.Lift1.setPower(0); //Stopped Lift
                if(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                    robot.Lift1.setPower(CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), LiftTargetHieght, 0.02));
//                    telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
//                    telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
//                    telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
//                    telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
//                    telemetry.update();
                }
                if(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                    robot.Lift2.setPower(CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), LiftTargetHieght, 0.02));
//                    telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
//                    telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
//                    telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
//                    telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
//                    telemetry.update();
                }
                if (LiftTargetHieght > 2500) {
                robot.Lift1.setPower(0);
                robot.Lift2.setPower(0);
              }
                if (LiftTargetHieght < 250) {
                   // robot.Lift1.setPower(0);
                   // robot.Lift2.setPower(0);
                    CustomLiftPID.KP = 0;
                    CustomLiftPID.KI = 0;
                    CustomLiftPID.KD = 0;
                    CustomLiftPID.KG = 0;
                    CustomLiftPID.IntRange = 0;
                }
                else {
                    CustomLiftPID.KP = 0.0005;
                    CustomLiftPID.KI = 0;
                    CustomLiftPID.KD = 0;
                    CustomLiftPID.KG = 0;
                    CustomLiftPID.IntRange = 0;

                }


                //OutTake
                if (gamepad2.left_bumper) {
                    robot.Dropper.setPosition(robot.Open);
                }
                else {
                    robot.Dropper.setPosition(robot.Closed);
                }

                if (gamepad2.dpad_down) {
                    gamepad2.isRumbling();
                    robot.Claw.setPosition(robot.ClawTurnGrab);
                }
                else if (gamepad2.dpad_up) {
                    gamepad2.isRumbling();
                    robot.Claw.setPosition(robot.ClawTurnStart);
                }

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

                if (gamepad1.left_bumper) {
                    robot.Launcher.setPosition(robot.LAUNCH);
                }
                else {
                    robot.Launcher.setPosition(robot.NO_LAUNCH);
                }

                 if (robot.Lift_Sensor.isPressed()) {
                    robot.Lift1.setPower(0);
                    robot.Lift2.setPower(0);
                }

                telemetry.addData("Red1", robot.OutTakeDetector1.red());
                telemetry.addData("Green1", robot.OutTakeDetector1.green());
                telemetry.addData("Blue1", robot.OutTakeDetector1.blue());
                telemetry.addData("Red2", robot.OutTakeDetector2.red());
                telemetry.addData("Green2", robot.OutTakeDetector2.green());
                telemetry.addData("Blue2", robot.OutTakeDetector2.blue());
                telemetry.update();

//                telemetry.addData("Lifting", -gamepad2.left_stick_y);
//                telemetry.addData("Lift1 target Height:", 560);
//                telemetry.addData("Lift1 Current Height:", robot.Lift1.getCurrentPosition());
//                telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift1.getCurrentPosition(), 560, 0.2));
//                telemetry.addData("Lift2 target Height:", 560);
//                telemetry.addData("Lift2 Current Height:", robot.Lift2.getCurrentPosition());
//                telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift2.getCurrentPosition(), 560, 0.2));
//                telemetry.update();
//                boolean isLeft = robot.OutTakeDetector2.red() > 500 && robot.OutTakeDetector2.blue() > 500 && robot.OutTakeDetector2.green() > 500;
//                boolean isNothing = robot.OutTakeDetector1.red() > 400 && robot.OutTakeDetector1.blue() < 300 && robot.OutTakeDetector1.green() < 300 && robot.OutTakeDetector2.red() > 400 && robot.OutTakeDetector2.blue() < 300 && robot.OutTakeDetector2.green() < 300;
//                boolean isRight = robot.OutTakeDetector1.red() > 1000 && robot.OutTakeDetector1.blue() > 1000 && robot.OutTakeDetector1.green() > 1000;
                boolean isYellowLeft = robot.OutTakeDetector1.red() > 1750 && robot.OutTakeDetector1.blue() > 500 && robot.OutTakeDetector1.green() > 2000 && robot.OutTakeDetector1.red() < 2000 && robot.OutTakeDetector1.blue() < 1000 && robot.OutTakeDetector1.green() < 3000;
                boolean isPurpleLeft = robot.OutTakeDetector1.red() > 1250 && robot.OutTakeDetector1.blue() > 2500 && robot.OutTakeDetector1.green() > 2000 && robot.OutTakeDetector1.red() < 1750 && robot.OutTakeDetector1.blue() < 3000 && robot.OutTakeDetector1.green() < 2200;
                boolean isGreenLeft = robot.OutTakeDetector1.red() > 250 && robot.OutTakeDetector1.blue() > 500 && robot.OutTakeDetector1.green() > 1500 && robot.OutTakeDetector1.red() < 750 && robot.OutTakeDetector1.blue() < 1000 && robot.OutTakeDetector1.green() < 2000;
                boolean isWhiteLeft = robot.OutTakeDetector1.red() > 2750 && robot.OutTakeDetector1.blue() > 4500 && robot.OutTakeDetector1.green() > 5250 && robot.OutTakeDetector1.red() < 4000 && robot.OutTakeDetector1.blue() < 5500 && robot.OutTakeDetector1.green() < 6000;
                boolean isYellowRight = robot.OutTakeDetector1.red() > 1750 && robot.OutTakeDetector1.blue() > 500 && robot.OutTakeDetector1.green() > 2000 && robot.OutTakeDetector1.red() < 2000 && robot.OutTakeDetector1.blue() < 1000 && robot.OutTakeDetector1.green() < 3000;
                boolean isPurpleRight = robot.OutTakeDetector2.red() > 1250 && robot.OutTakeDetector2.blue() > 2500 && robot.OutTakeDetector2.green() > 2000  && robot.OutTakeDetector2.red() < 1750 && robot.OutTakeDetector2.blue() < 3000 && robot.OutTakeDetector2.green() < 2200;
                boolean isGreenRight = robot.OutTakeDetector2.red() > 250 && robot.OutTakeDetector2.blue() > 500 && robot.OutTakeDetector2.green() > 1500 && robot.OutTakeDetector2.red() < 750 && robot.OutTakeDetector2.blue() < 1000 && robot.OutTakeDetector2.green() < 2000;
                boolean isWhiteRight = robot.OutTakeDetector2.red() > 2750 && robot.OutTakeDetector2.blue() > 4500 && robot.OutTakeDetector2.green() > 5250 && robot.OutTakeDetector2.red() < 3250 && robot.OutTakeDetector2.blue() < 5500 && robot.OutTakeDetector2.green() < 6000;
                    if (isYellowLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }
                    else if (isPurpleLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                    }
                    else if (isGreenLeft) {
                         revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    else if (isWhiteLeft) {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    }
                    else {
                        revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }

                if (isYellowRight) {
                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
                else if (isPurpleRight) {
                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
                else if (isGreenRight) {
                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                else if (isWhiteRight) {
                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                }
                else {
                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

//                if (isRight) {
//                    revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
//                }
//                else if (isLeft) {
//                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
//                }
//                else if (isLeft && isRight) {
//                    revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                    revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                }
//                else {
//                  revBlinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                  revBlinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            }


            }

        }


    }

    private void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        robot.leftFront.setPower(wheels.leftFront * DrivePower);
        robot.rightFront.setPower(wheels.rightFront * DrivePower);
        robot.leftBack.setPower(wheels.leftBack * DrivePower);
        robot.rightBack.setPower(wheels.rightBack * DrivePower);
    }

}

