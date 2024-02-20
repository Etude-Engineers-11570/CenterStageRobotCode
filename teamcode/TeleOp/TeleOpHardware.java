/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
modification,
 * are permitted (subject to the limitations in the disclaimer below)
provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be
used to endorse or
 * promote products derived from this software without specific prior
written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;


//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.hardware.SensorColor;

import android.graphics.Color;
import android.hardware.Sensor;
import android.text.method.Touch;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Serializable;
//import com.arcrobotics.ftclib.controller.PIDFController;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a
 single robot.
 */
public class TeleOpHardware
{
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();
    /* Public OpMode members. */

    //Drive Motors:
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;

    //Outtake
    public Servo Dropper = null;
    public Servo Dropper_Turn1 = null;
    public Servo Dropper_Turn2 = null;

    double Closed = 0;
    double Open = 0.1;
    double Straight1 = 0.2;
    double Straight2 = 0.2;
    double Turn1 = 0.533333333333333333;
    double Turn2 = 0.533333333333333333;

    double Turn1Degrees = Turn1 * 300.0;
    double Turn2Degrees = Turn2 * 300.0;

    //AirPlane Launcher
    double LAUNCH = 0.5;
    double NO_LAUNCH = 0;

    public Servo Launcher = null;

    public Servo Claw;

    double ClawTurnGrab = 0.38;
    double ClawTurnStart = 0.01;




    //Sensors
    public ColorSensor OutTakeDetector1 = null;
    public ColorSensor OutTakeDetector2;
    public TouchSensor Lift_Sensor;

    //Scoring Pixels:
    //Intake:
    public DcMotor Intake = null;

    //Lift:
    public DcMotor Lift1;
    public DcMotor Lift2;

    //Drone_Launcher:
    public Servo Drone_Launcher = null;

    public DcMotor Hook = null;

    /* local OpMode members. */


    /* Constructor */
    public TeleOpHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap)
    {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        // Define and Initialize Motors

        //Drive Motors:
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");


        //Outtake
         Dropper = hwMap.get(Servo.class, "Dropper");
         Dropper_Turn1 = hwMap.get(Servo.class, "Dropper_Turn1");
         Dropper_Turn2 = hwMap.get(Servo.class, "Dropper_Turn2");
        //Scoring Pixels:
        //Intake:
        Intake  = hwMap.get(DcMotor.class, "Intake");

        Hook = hwMap.get(DcMotor.class, "Hook");

        //Lift
        Lift1 = hwMap.get(DcMotor.class,"Lift1");
        Lift2 = hwMap.get(DcMotor.class, "Lift2");

        //AirPlane Launcher
        Launcher = hwMap.get(Servo.class, "Launcher");

        Claw = hwMap.get(Servo.class, "Claw");
        // Drone_Launcher
       // Drone_Launcher = hwMap.get(Servo.class, "Launcher");

        //Initialize Robot:

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
       Intake.setDirection(DcMotor.Direction.REVERSE);
        Lift1.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
      //  Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher.setDirection(Servo.Direction.FORWARD);
        Dropper_Turn2.setDirection(Servo.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Claw.setPosition(ClawTurnStart);




        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        Intake.setPower(0);
        Hook.setPower(0);
        Lift1.setPower(0);
        Lift2.setPower(0);


        Dropper.setPosition(Closed);
        Dropper_Turn1.setPosition(Straight1);
        Dropper_Turn2.setPosition(Straight2);
        Launcher.setPosition(NO_LAUNCH);

        // Sensors

        Lift_Sensor = hwMap.get(TouchSensor.class, "Lift_Sensor");


        OutTakeDetector1 = hwMap.get(ColorSensor.class, "OutTakeDetector1");
        OutTakeDetector2 = hwMap.get(ColorSensor.class, "OutTakeDetector2");
    }


        // Do something with the color values



    }
