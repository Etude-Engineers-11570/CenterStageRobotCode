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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    public void MoveDriveTrain() {
        double Vertical;
        double Horizontal;
        double Pivot;

        Vertical = gamepad1.left_stick_x;
        Horizontal = gamepad1.left_stick_y;
        Pivot = gamepad1.right_stick_x;

        robot.FR.setPower(-Pivot + (Vertical - Horizontal));
        robot.BR.setPower(Pivot + (Vertical + Horizontal));
        robot.FL.setPower(-Pivot + (-Vertical + Horizontal));
        robot.BL.setPower(Pivot + (-Vertical - Horizontal));

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
    double Multiplyer = 3;
    double DrivePower = 1.0;
    double IntakeSpeed = 1;

    @Override
    public void runOpMode() {

        robot.FL = hardwareMap.get(DcMotor.class, "FL");
        robot.FR = hardwareMap.get(DcMotor.class, "FR");
        robot.BL = hardwareMap.get(DcMotor.class, "BL");
        robot.BR = hardwareMap.get(DcMotor.class, "BR");

        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MoveDriveTrain();
            //Always running:
            DrivePower = 1 / ( 1 + gamepad1.right_trigger * Multiplyer ); //lowest speed is 1/1+multiplyer, rn 1/4


           /* if(gamepad2.left_stick_y>0)
            {
                robot.Lift.setPower(0); //When we are all the way down we the intake automatically
                LiftTargetHieght = robot.Lift.getCurrentPosition();
            }
            if (-gamepad2.left_stick_y > 0.025 || -gamepad2.left_stick_y < -0.025) {
                robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                LiftTargetHieght = robot.Lift.getCurrentPosition();
            }

            else
            {
                robot.Lift.setPower(0); //Stopped Lift
                if(CustomLiftPID.Calculate(robot.Lift.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                    robot.Lift.setPower(CustomLiftPID.Calculate(robot.Lift.getCurrentPosition(), LiftTargetHieght, 0.02));
                } */

            //OutTake
           /* if (gamepad2.left_bumper) {
                robot.Dropper.setPosition(robot.Open);
            }
            else {
                robot.Dropper.setPosition(robot.Closed);
            }
            //Intake
            if (gamepad2.left_stick_y > 0.025) {
            robot.Intake.setPower(gamepad1.right_stick_y * IntakeSpeed); }
            else {
                robot.Intake.setPower(0);
            } */


        }

    }


}

