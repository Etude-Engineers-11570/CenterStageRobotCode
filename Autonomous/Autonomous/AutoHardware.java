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

package org.firstinspires.ftc.teamcode.Autonomous;


//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.hardware.SensorColor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDFController;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a
 single robot.
 */
public class AutoHardware
{
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();
    /* Public OpMode members. */

    //Drive Motors:
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    //Outtake
    // public Servo Dropper = null;

    double Closed = 0;
    double Open = 1;


    //Scoring Pixels:
    //Intake:
    // public DcMotor Intake = null;

    //Lift:
    // public DcMotor Lift = null;


    /* local OpMode members. */


    /* Constructor */
    public AutoHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap)
    {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        // Define and Initialize Motors

        //Drive Motors:
        FL  = hwMap.get(DcMotor.class, "FL");
        BL = hwMap.get(DcMotor.class, "BL");
        FR  = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");


        //Outtake
        // Dropper = hwMap.get(Servo.class, "Dropper");
        //Scoring Pixels:
        //Intake:
        // Intake  = hwMap.get(DcMotor.class, "Intake");

        //Lift
        //Lift = hwMap.get(DcMotor.class,"Lift");

        //Sensors:

        //LED
        // RevBlinkinLedDriver revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class,"LED");
        // revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);

        //Initialize Robot:

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        // Intake.setDirection(DcMotor.Direction.REVERSE);
        // Lift.setDirection(DcMotor.Direction.FORWARD);
        //Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Set all motors to zero power
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        // Intake.setPower(0);
        // Lift.setPower(0);


        // Dropper.setPosition(Closed);

    }
}
