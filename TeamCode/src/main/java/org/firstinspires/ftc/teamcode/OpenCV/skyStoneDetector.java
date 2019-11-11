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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint({"DefaultLocale","SdCardPath"})
public class skyStoneDetector extends Thread {

    // File Paths
    private final String basePath = "/sdcard/FIRST/openCV/";
    private final String openClosePath = basePath + "openClose";
    private final String croppedPath = basePath + "croppedImage";
    private final String verViewPath = basePath + "verticalAvg";
    private final String testPath = "/sdcard/FIRST/testFiles/";

    private final String hsvPath = basePath + "hsv";
    private final String filPath = basePath + "satfil";
    private final String HPath = basePath + "h";
    private final String SPath = basePath + "s";
    private final String VPath = basePath + "v";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------

    private final int horThreshold = 90;
    private final int verThreshold = 225;
    private final int magnificationFactor = 10;
    private final int binaryValue = 255;
    private final int columnsBack = 10;
    private final int columnDiff = 75;

    // quarry row 48in from wall, robot is 18in, phone 30in away...
    // stone width ~55px, half of stone width ~27.5px, adjust to 20
    private final double stoneWidth = 20;

    private int frameNum = 1;
    private double leftSSCenter = -1;

    private LinearOpMode op;
    public skyStoneDetector(LinearOpMode opMode) {op = opMode;}

    private double stoneSum = 0;
    private double curStoneCount;
    private boolean autoActive = false;

    public void initializeCamera() {
        telemetry2("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) FtcRobotControllerActivity.showCameraPreview();
        telemetry2("Status", "Ready");
    }

    @Override public void run() {
        setName("OpenCV");

        File dir = new File(basePath);
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {new File(dir, child).delete();}
        }

        if (usingCamera) {
            FtcRobotControllerActivity.endPreview();
            frameGrabber = FtcRobotControllerActivity.frameGrabber;

            while (autoActive) {
                Mat input = frameGrabber.getNextMat();

                if (input != null) {
                    log("Frame " + frameNum + " ----------------------------------------");
                    leftSSCenter = detectSkyStone(input);
                    log("Left SkyStone Position: " + leftSSCenter);
                    frameNum++;
                } else leftSSCenter = -1;
            }
            log("Avg stone is view: " + String.format("%.2f", stoneSum /frameNum));
            FtcRobotControllerActivity.disableCameraView();
        } else {
            Mat in = Imgcodecs.imread(testPath + "test.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(in, in, new Size(240, 180));
            leftSSCenter = detectSkyStone(in);
        }
        log(" ");
    }

    private double detectSkyStone (Mat input) {
        double leftSSPos = -1;

        // Convert to HSV (Saturation)
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
        //if (frameNum<200) Imgcodecs.imwrite(hsvPath + frameNum + ".jpg", HSV);

        List<Mat> hsvTypes = new ArrayList<>(3);
        Core.split(HSV, hsvTypes);
        Mat satUnfiltered = hsvTypes.get(1);
        /*if (frameNum<200) Imgcodecs.imwrite(HPath + frameNum + ".jpg", hsvTypes.get(0));
        if (frameNum<200) Imgcodecs.imwrite(SPath + frameNum + ".jpg", satUnfiltered);
        if (frameNum<200) Imgcodecs.imwrite(VPath + frameNum + ".jpg", hsvTypes.get(2));*/

        // Filter Saturation Image
        Mat satFiltered = new Mat();
        Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 135, 10), satFiltered);
        //if (frameNum<200) Imgcodecs.imwrite(filPath + frameNum + ".jpg", satFiltered);

        // Remove extraneous data
        Mat openClose = new Mat();
        Imgproc.morphologyEx(satFiltered, openClose, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(openClose, openClose, Imgproc.MORPH_CLOSE, new Mat());
        if (frameNum < 200) Imgcodecs.imwrite(openClosePath + frameNum + ".jpg", openClose);

        // Crop Image to where quarry row is
        double horAvg;
        Mat SCropped = new Mat();
        for (int row = 0; row < openClose.rows(); row++) {
            horAvg = Core.mean(openClose.row(row)).val[0];
            if (horAvg > horThreshold) SCropped.push_back(openClose.row(row));
        }

        if (!(SCropped.cols() == 0)) {
            //if (frameNum<200) Imgcodecs.imwrite(croppedPath + frameNum + ".jpg", SCropped);

            // Makes image black(stone) and white(skyStone)
            double verAvg;
            /**/ Mat verImage = new Mat(10, SCropped.cols(), CvType.CV_8UC1);
            for (int col = 0; col < SCropped.cols(); col++) {
                verAvg = Core.mean(openClose.col(col)).val[0] * magnificationFactor;
                if (verAvg <= verThreshold) verAvg = 0;
                else verAvg = binaryValue;
                verImage.col(col).setTo(new Scalar(verAvg));
            }
            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());
            if (frameNum < 200) Imgcodecs.imwrite(verViewPath + frameNum + ".jpg", verImage);

            /*String verCols = "";
            for (int col = 0; col < verImage.cols(); col++) {
                verCols += (new Scalar(verImage.get(0, col)).val[0]) + ", ";
            }
            log("Vertical: " + verCols);*/

            // Image Analyzing
            //ArrayList<Integer> darkCols = new ArrayList<>();
            ArrayList<Double> darkAreas = new ArrayList<>();
            double prevIntensity = 0;
            for (int c = 0; c < verImage.cols(); c++) {
                double curIntensity = new Scalar(verImage.get(0, c)).val[0];

                if (curIntensity == 0) {
                    //darkCols.add(c);

                    double intensityDiff;
                    if (c < columnsBack) intensityDiff = Math.abs(curIntensity - prevIntensity);
                    else intensityDiff =
                            Math.abs(curIntensity - new Scalar(verImage.get(0, c - columnsBack)).val[0]);

                    if (intensityDiff == binaryValue) darkAreas.add(c + stoneWidth);
                }
                prevIntensity = curIntensity;
            }
            //log("Dark Columns: " + darkCols);
            //log(darkAreas.size() + " Dark Areas: " + darkAreas);

            double prevArea = 0;
            boolean firstStone = true;
            for (int a = 0; a < darkAreas.size(); a++) {
                double curArea = darkAreas.get(a);

                double areaDiff = Math.abs(curArea - prevArea);
                if (areaDiff < columnDiff && !firstStone) {
                    darkAreas.remove(a);
                    a--;
                }
                firstStone = false;

                prevArea = curArea;
            }
            log(darkAreas.size() + " Dark Areas: " + darkAreas);
            log("SkyStones Detected: " + darkAreas.size());
            curStoneCount = darkAreas.size();
            stoneSum += curStoneCount;

            if (!(darkAreas.size() == 0)) {
                leftSSPos = darkAreas.get(0);
            } else log("Cannot Determine Left SkyStone Position :-(");
        } else {
            log("Quarry Row Not Detected :-(");
        }
        return leftSSPos;
    }

    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }

    private void log(String message) {Log.w("opencv-main", message);}

    public double getPosition() {return leftSSCenter;}

    public double getNumberOfStones() {return curStoneCount;}

    public void setAutoActive(boolean autoActive) {
        this.autoActive = autoActive;
    }
}