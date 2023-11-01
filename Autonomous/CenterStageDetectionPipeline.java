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

/*
package org.firstinspires.ftc.teamcode.Autonomous;;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous

public class CenterStageDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(100, 35),
            new Point(160, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public CenterStageDetectionPipeline (Telemetry t) { telemetry = t; }

    /* Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.MIDDLE;
            telemetry.addData("Skystone Location", "Middle");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar StoneColor = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:StoneColor);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:StoneColor);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorSkystone:StoneColor);

        return mat;
    }

    public Location getLocation() {
        return location;
    } */
/*
    @Override
    public Mat processFrame(Mat input) {
        // Convert input to the appropriate color space if needed
        // Define HSV range for white detection
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        // Apply inRange function to isolate white regions
        Core.inRange(input, lowHSV, highHSV, mat);

        Mat areaA = mat.submat(MIDDLE_ROI);
        Mat areaB = mat.submat(LEFT_ROI);
        Mat areaC = mat.submat(RIGHT_ROI);

        double intensityA = Core.sumElems(areaA).val[0] / MIDDLE_ROI.area() / 255;
        double intensityB = Core.sumElems(areaB).val[0] / LEFT_ROI.area() / 255;
        double intensityC = Core.sumElems(areaC).val[0] / RIGHT_ROI.area() / 255;

        areaA.release();
        areaB.release();
        areaC.release();

        // Determine the area with the highest white intensity
        if (intensityA > intensityB && intensityA > intensityC) {
            location = Location.MIDDLE;
        } else if (intensityB > intensityA && intensityB > intensityC) {
            location = Location.LEFT;
        } else {
            location = Location.RIGHT;
        }

        Scalar colorWhite = new Scalar(255, 255, 255);  // White color in BGR format
        Scalar colorOther = new Scalar(0, 0, 255);     // A different color (e.g., red) in BGR format


        // Draw rectangles on the frame to highlight the ROIs and the detected location
        Imgproc.rectangle(input, MIDDLE_ROI, location == Location.MIDDLE ? colorWhite : colorOther);
        Imgproc.rectangle(input, LEFT_ROI, location == Location.LEFT ? colorWhite : colorOther);
        Imgproc.rectangle(input, RIGHT_ROI, location == Location.RIGHT ? colorWhite : colorOther);

        // Return the modified input image
        return input;
    }

} */