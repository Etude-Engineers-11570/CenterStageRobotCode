package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStage_Detection_Pipeline_1 extends OpenCvPipeline
{
    public enum PropPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    static final Scalar BLUE = new Scalar(100, 200, 194.1);
    static final Scalar RED = new Scalar(35.9, 149.2, 5.7);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,185);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(85,160);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(285,185);

    static final int REGION_WIDTH = 30;
    static final int REGION_HEIGHT = 30;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y); //A is same as top left)
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    public Mat region1= new Mat();
    public Mat region2= new Mat();
    public Mat region3= new Mat();
    //Unused:
    public Mat customColorSpace = new Mat();
        public Mat Cr = new Mat();
        public Mat YCrCb = new Mat();
        public Mat Cb = new Mat();
    //Whole image:
    public Mat WholeImage = new Mat();
    //Averages:
    int avg1, avg2, avg3;
    private volatile PropPosition position = PropPosition.LEFT;
      public void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }
        public void inputToCr(Mat input) {
            Imgproc.cvtColor(input, customColorSpace, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(customColorSpace, Cr, 0); // Extracts the Cr channel (red)
    }

    @Override
    public void init(Mat firstFrame)
    {

            inputToCr(firstFrame);
            inputToCb(firstFrame);

        WholeImage = firstFrame; //Sets the matrix to the matrix gotten by the webcam (image)
        region1 = WholeImage.submat(new Rect(region1_pointA, region1_pointB));
        region2 = WholeImage.submat(new Rect(region2_pointA, region2_pointB));
        region3 = WholeImage.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input)
    {

             inputToCr(input);
             inputToCb(input);

        avg1 = (int) Core.mean(region1).val[1];
        avg2 = (int) Core.mean(region2).val[1];
        avg3 = (int) Core.mean(region3).val[1];


        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines */

        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

       if(max == avg1) // Was it from region 1?
        {
            position = PropPosition.LEFT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        if(max == avg2) // Was it from region 2?
        {
            position = PropPosition.CENTER; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = PropPosition.RIGHT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else {
            position = PropPosition.LEFT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        return input;
    }
    public PropPosition getAnalysis()
    {
        return position;
    }
}