package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStage_Detection_Pipeline_4 extends OpenCvPipeline
{
    Telemetry telemetry;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 60),
            new Point(90, 150));
    static final Rect MID_ROI = new Rect(
            new Point(110, 50),
            new Point(200, 140));
    static final Rect RIGHT_ROI = new Rect(
            new Point(230, 60),
            new Point(320, 150));
    public String ObjectDirection;
    Mat mat = new Mat();
    Mat thresh = new Mat();
    Mat left, right, mid;

    public CenterStage_Detection_Pipeline_4(Telemetry t, String s) {
        telemetry = t;
        ObjectDirection = s;
    }

    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //Uses HSV Colors

        Scalar lowHSVRed = new Scalar(0,100,20); // lower bound HSV for red 0 100 20
        Scalar highHSVRed = new Scalar(15, 255, 255); // higher bound HSV for red 10 255 255

//        Scalar lowHSVBlue = new Scalar(85, 123, 0); // lower bound HSV for blue 110 100 20
//        Scalar highHSVBlue = new Scalar(177, 255, 255); // higher bound HSV for blue 130 255 255

        thresh.release();


        Core.inRange(mat, lowHSVRed, highHSVRed, thresh);
        //Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);

        left = thresh.submat(LEFT_ROI);
        right = thresh.submat(RIGHT_ROI);
        mid = thresh.submat(MID_ROI);




        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();
        mid.release();

        //telemetry.addData("Left raw value", Core.sumElems(left).val[0]);
        //telemetry.addData("Right raw value",  Core.sumElems(right).val[0]);
        //telemetry.addData("Mid raw value", Core.sumElems(mid).val[0]);

        //telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        //telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        //telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");


        double maximum=Math.max(leftValue, Math.max(midValue, rightValue));
        boolean objLeft = leftValue ==maximum;
        boolean objRight = rightValue==maximum;
        boolean objMid = midValue==maximum;

        if(objLeft){
            ObjectDirection = "LEFT";
            Imgproc.rectangle(
                    thresh, //mat
                    LEFT_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else if(objRight){
            ObjectDirection = "RIGHT";
            Imgproc.rectangle(
                    thresh, //mat
                    RIGHT_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else if(objMid){
            ObjectDirection = "MIDDLE";
            Imgproc.rectangle(
                    thresh, //mat
                    MID_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else{
            ObjectDirection = "NONE";
        }


        //telemetry.addData("Location: ", ObjectDirection);



        return thresh;

    }
    public String getPosition(){
        return ObjectDirection;
    }
}