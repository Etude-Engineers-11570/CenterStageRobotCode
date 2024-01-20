package org.firstinspires.ftc.teamcode.Autonomous;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * **FOR CALIBRATING THE CAMERA ONLY**
 *
 * Set up:
 * Download & run EOCVSim (<a href="https://github.com/deltacv/EOCV-Sim/releases">...</a>).
 * On the right menu create & select a workspace: Workspace -> Select Workspace -> Navigate & select the "vision" package/folder in this project.
 * On the right menu create & select a webcam source. For competition directly connect to the usb webcam on the robot.
 *
 * Usage:
 * On default the camera will cycle through: No mask (-1) -> Red mask (0) -> Blue mask (1). To select a single mask to work on, change the "setMask" integer on the bottom menu (default -2, -1, 0, or 1).
 * At the bottom, you will tune the LOWER_RED, and LOWER_BLUE sliders.
 * On the right read the telemetry on each possible game element position to also tune the DETECTION_THRESHOLD.
 * ** Note that these values will not save and will need to be copied into the Constants.CameraConstants class. **
 */
public class CenterStage_Detection_Pipeline_5 extends OpenCvPipeline {
    public double DETECTION_THRESHOLD = 0.7;
    public Scalar LOWER_RED = new Scalar(79, 91, 167);
    public Scalar LOWER_BLUE = new Scalar(23, 78, 184);
    Scalar UPPER = new Scalar(255, 255, 255);
    public int setMask = -2;

    String gameElementPosition = "";
    int cameraWidth;
    int cameraHeight;
    int frameCount = 0;
    int currentPregameMask = -1;
    Mat input = new Mat();
    final Mat ycrcbMat = new Mat();
    final Mat binaryMat = new Mat();
    Mat maskedInputMat = new Mat();
    final Mat pregameMaskedInputMat = new Mat();
    Scalar lower;
    int color;
    Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {
        this.input = input;
        frameCount++;
        if (frameCount < 25){ addPreGameColorMask(currentPregameMask); }
        else if (currentPregameMask < 0.5){
            frameCount = 0;
            currentPregameMask++;
        } else {
            frameCount = 0;
            currentPregameMask = -1;
        }
        return pregameMaskedInputMat;
    }

    public CenterStage_Detection_Pipeline_5 (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public String findGameElement(int color){ // 0: RED, 1: BLUE
        cameraWidth = input.width();
        cameraHeight = input.height();

        addColorMask(color);

        Mat leftZone = maskedInputMat.submat(new Rect(0, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor1 = Core.mean(leftZone);
        telemetry.addData("Middle Confidence", avgColor1.val[0]);

        Mat rightZone = maskedInputMat.submat(new Rect(cameraWidth / 2, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor2 = Core.mean(rightZone);
        telemetry.addData("Right Confidence", avgColor2.val[0]);

        telemetry.addData("Game element position", gameElementPosition);
        telemetry.addData("Current mask", currentPregameMask);
        telemetry.update();

        if (avgColor1.val[0] > DETECTION_THRESHOLD) return "Middle";
        if (avgColor2.val[0] > DETECTION_THRESHOLD) return "Right";
        return "Left";
    }

    private void addColorMask(int color){ // 0: RED, 1: BLUE
        if (color != -1) {
            lower = (color == 0) ? LOWER_RED : LOWER_BLUE;
        } else {
            lower = new Scalar(0, 0, 0);
        }
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, UPPER, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
    }

    private void addPreGameColorMask(int current){ // -1: No mask, 0: RED, 1: BLUE
        if (current == -1){
            color = -1;
            lower = new Scalar(0, 0, 0);
        } else if (current == 0){
            color = 0;
            lower = LOWER_RED;
        } else if (current == 1){
            color = 1;
            lower = LOWER_BLUE;
        }

        if (setMask == -1){
            color = -1;
            lower = new Scalar(0, 0, 0);
        } else if (setMask == 0){
            color = 0;
            lower = LOWER_RED;
        } else if (setMask == 1){
            color = 1;
            lower = LOWER_BLUE;
        }

        gameElementPosition = findGameElement(color);
        telemetry.update();
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, UPPER, binaryMat);
        pregameMaskedInputMat.release();
        Core.bitwise_and(input, input, pregameMaskedInputMat, binaryMat);
    }
}