package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.FrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * This class processes camera frames from {@linkplain FrameGrabber}
 * to determine the location of stones in autonomous
 */

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint({"DefaultLocale","SdCardPath"})
public class stoneLocator2 extends Thread {

    // Variables
    private final static String basePath = "/sdcard/FIRST/procFiles3/";
    private final static String inputPath = basePath + "input";
    private final static String filteredPath = basePath + "filtered";
    private final static String contoursPath = basePath + "contours";
    private final static String circlePath = basePath + "circle";
    private final static String ellipsePath = basePath + "ellipse";
    private final static String testPath = "/sdcard/FIRST/testFiles3/";
    private ElapsedTime timer = new ElapsedTime();

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = false; // <<<----------------------
    private final boolean debug = true;

    private int frameNum = 1;
    private double[] sPos = {-1, -1, -1};
    private double time = -1;

    private boolean active = false;

    private LinearOpMode op;
    public stoneLocator2(LinearOpMode opMode) {op = opMode;}

    // Phone Position-
    // 7in up, side closest to camera is 7.5in from left of robot (aligned to depot), slight tilt forward

    // difference between 2 skystones is 170
    
    /**
     * Enables the camera view
     */
    public void initializeCamera() {
        telemetry2("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) FtcRobotControllerActivity.enableCameraView();
        telemetry2("Status", "Ready");
    }
    
    /**
     * Runs OpenCV Thread
     */
    @Override public void run() {
        setName("OpenCV2");

        // Clear Image Folder
        File dir = new File(basePath);
        String[] children = dir.list();
        if (children != null) {for (String child : children) {new File(dir, child).delete();}}

        // Use and Process Camera Image, or Use and Process Hardcoded Image
        if (usingCamera) {
            frameGrabber = FtcRobotControllerActivity.frameGrabber;

            while (active) {
                Mat input = frameGrabber.getNextMat();
                if (input != null) {
                    log("Frame " + frameNum + " ----------------------------------------");
                    sPos = detectSkyStone(input);
                    frameNum++;
                } else sPos = new double[]{-1, -1, -1};
            }
            FtcRobotControllerActivity.disableCameraView();
        } else {
            Mat input = Imgcodecs.imread(testPath + "test5.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
            frameNum++;

            input = Imgcodecs.imread(testPath + "test6.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
            frameNum++;

            input = Imgcodecs.imread(testPath + "test7.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
        }
        log("");
    }
    
    /**
     * Finds position value of stone by processing an input frame
     * @param input the camera frame that will be used to detect skystones
     * @return x, y, and theta of stone
     */
    private double[] detectSkyStone (Mat input) {
        // Log Input Image and Reset Variables and Timer
        timer.reset();
        double stoneX = -1;
        double stoneY = -1;
        double stoneTheta = -1;
        if (debug) {Imgcodecs.imwrite(inputPath + (frameNum % 100) + ".jpg", input);}

        // Process Image
        Mat filtered = new Mat(180, 240, CvType.CV_8UC1, new Scalar(255));
        Imgproc.cvtColor(input, filtered, Imgproc.COLOR_RGB2HSV);
        Core.inRange(filtered, new Scalar(85, 90, 90), new Scalar(115, 255, 255), filtered);
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_CLOSE, new Mat());
        if (debug) Imgcodecs.imwrite(filteredPath + (frameNum % 100) + ".jpg", filtered);

        // Further Process Image
        Imgproc.Canny(filtered, filtered, 0, 0, 3, false);
        if (debug) Imgcodecs.imwrite(contoursPath + (frameNum % 100) + ".jpg", filtered);

        // Find Contours
        Mat heirarchyMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(filtered, contours, heirarchyMat, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);

        // Find Bottommost Point
        int contourIndex = 0;
        for (int i = 0; i < contours.size(); i++) {
            for (int j = 0; j < contours.get(i).rows(); j++) {
                if (contours.get(i).get(j, 0)[1] >= stoneY) {
                    stoneX = contours.get(i).get(j, 0)[0];
                    stoneY = contours.get(i).get(j, 0)[1];
                }
            }
        }
        Imgproc.circle(input, new Point(stoneX, stoneY), 2, new Scalar(0, 0, 255), 2);
        if (debug) Imgcodecs.imwrite(circlePath + (frameNum % 100) + ".jpg", input);
        //log("t2: " + timer.milliseconds());

        // Find Ellipse Using Contour Index
        Mat ellipseOnly = new Mat();
        RotatedRect ellipse;
        ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(contourIndex).toArray()));
        stoneTheta = ellipse.angle;
        Imgproc.ellipse(ellipseOnly, ellipse, new Scalar(0), 1);
        if (debug) Imgcodecs.imwrite(ellipsePath + (frameNum % 100) + ".jpg", ellipseOnly);

        // Log and Return Data
        time = timer.milliseconds();
        log("x: " + stoneX);
        log("y: " + stoneY);
        log("theta: " + stoneTheta);
        log("ms: " + time);
        return new double[] {stoneX, stoneY, stoneTheta};
    }
    
    /**
     * Gets current stone position value (x, y, theta)
     * @return current stone position value (x, y, theta)
     */
    public double[] getLocation() {return sPos;}

    /**
     * Gets current frame process time (ms)
     * @return current frame process time (ms)
     */
    public double getTime() {return time;}
    
    /**
     * Sets whether the stone locator is actively processing camera frames to locate stone
     * @param active true = processing; false = not processing
     */
    public void setActive(boolean active) {this.active = active;}

    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }
    
    private void log(String message) {Log.w("opencv-sl", message);}
}