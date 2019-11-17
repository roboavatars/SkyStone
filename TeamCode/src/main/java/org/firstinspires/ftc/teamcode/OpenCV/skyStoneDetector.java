package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    private final String basePath = "/sdcard/FIRST/procFiles/";
    private final String satFilteredPath = basePath + "sFiltered";
    private final String openClosePath = basePath + "openClose";
    private final String croppedPath = basePath + "croppedImage";
    private final String verViewPath = basePath + "verticalAvg";
    private final String testPath = "/sdcard/FIRST/testFiles/";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------

    private final int horThreshold = 90;
    private final int verThreshold = 225;
    private final int magnificationFactor = 10;
    private final int binaryValue = 255;

    private int frameNum = 1;
    private double ssPos = -1;
    private double ssXPos = -1;

    private boolean active = false;
    private double stoneSum = 0;
    private double curStoneCount;
    private double stoneLength;
    private boolean isRed = true;

    private LinearOpMode op;
    public skyStoneDetector(LinearOpMode opMode) {op = opMode;}

    // Phone Position-
    // 7in up, side closest to camera is 7.5in from left of robot (aligned to depot), slight tilt forward

    // difference between 2 skystones is 170

    public void initializeCamera() {
        telemetry2("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) FtcRobotControllerActivity.enableCameraView();
        telemetry2("Status", "Ready");
    }

    @Override public void run() {
        setName("OpenCV");

        // clear folder of images
        File dir = new File(basePath);
        String[] children = dir.list();
        if (children != null) {for (String child : children) {new File(dir, child).delete();}}

        if (usingCamera) {
            frameGrabber = FtcRobotControllerActivity.frameGrabber;

            while (active) {
                Mat input = frameGrabber.getNextMat();

                if (input != null) {
                    log("Frame " + frameNum + " ----------------------------------------");
                    ssPos = detectSkyStone(input);
                    log("SkyStone Position: " + ssPos);
                    frameNum++;
                } else ssPos = -1;
            }
            log("Avg stone is view: " + String.format("%.2f", stoneSum /frameNum));
            FtcRobotControllerActivity.disableCameraView();
        } else {
            Mat in = Imgcodecs.imread(testPath + "test.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(in, in, new Size(240, 180));
            ssPos = detectSkyStone(in);
        }
        log(" ");
    }

    private double detectSkyStone (Mat input) {
        double ssPosValue = -1;

        // Convert to HSV (Saturation)
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
        List<Mat> hsvTypes = new ArrayList<>(3);
        Core.split(HSV, hsvTypes);
        Mat satUnfiltered = hsvTypes.get(1);

        // Filter Saturation Image
        Mat satFiltered = new Mat();
        Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 135, 10), satFiltered);
        // if (frameNum < 200) Imgcodecs.imwrite(satFilteredPath + frameNum + ".jpg", satFiltered);

        // Remove extra noise in image
        Mat openClose = new Mat();
        Imgproc.morphologyEx(satFiltered, openClose, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(openClose, openClose, Imgproc.MORPH_CLOSE, new Mat());
        //if (frameNum < 200) Imgcodecs.imwrite(openClosePath + frameNum + ".jpg", openClose);

        // Crop Image to where quarry row is
        double horAvg;
        Mat SCropped = new Mat();
        for (int row = 0; row < openClose.rows(); row++) {
            horAvg = Core.mean(openClose.row(row)).val[0];
            if (horAvg > horThreshold) SCropped.push_back(openClose.row(row));
        }

        if (!(SCropped.cols() == 0)) {
            if (frameNum < 200) Imgcodecs.imwrite(croppedPath + frameNum + ".jpg", SCropped);

            // Makes image black(stone) and white(skyStone)
            double verAvg;
            Mat verImage = new Mat(10, SCropped.cols(), CvType.CV_8UC1);
            for (int col = 0; col < SCropped.cols(); col++) {
                verAvg = Core.mean(openClose.col(col)).val[0] * magnificationFactor;
                if (verAvg <= verThreshold) verAvg = 0;
                else verAvg = binaryValue;
                verImage.col(col).setTo(new Scalar(verAvg));
            }
            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());
            if (frameNum < 200) Imgcodecs.imwrite(verViewPath + frameNum + ".jpg", verImage);

            /*String verCols = "";
            for (int col = 0; col < verImage.cols(); col++) {verCols+=(new Scalar(verImage.get(0, col)).val[0])+", ";}
            log("Vertical: " + verCols);*/

            // Image Analyzing
            ArrayList<Double> darkAreas = new ArrayList<>();
            double left = 0, right = 0, firstLeft = 0;
            double curIntensity;
            double nextIntensity = new Scalar(verImage.get(0, 0)).val[0];
            for (int c = 0; c < verImage.cols()-1; c++) {
                curIntensity = nextIntensity;
                nextIntensity = new Scalar(verImage.get(0, c+1)).val[0];

                if (curIntensity == binaryValue && nextIntensity == 0) {
                    left = c;
                    firstLeft = c;
                }
                else if (curIntensity == 0 && nextIntensity == binaryValue) {
                    right = c;
                    double stoneCenter = (left + right) / 2;
                    stoneLength = right - left;
                    darkAreas.add(stoneCenter);
                    left = 0; right = 0;
                }
            }

            /*if (darkAreas.size() == 1 && new Scalar(verImage.get(0, verImage.cols()-1)).val[0] == 0) {
                darkAreas.add(left+darkAreas.get(0)-firstLeft);
            }*/
            //log(darkAreas.size() + " Dark Areas: " + darkAreas);

            double prevArea = 0;
            boolean firstStone = true;
            for (int a = 0; a < darkAreas.size(); a++) {
                double curArea = darkAreas.get(a);

                double areaDiff = Math.abs(curArea - prevArea);
                if (areaDiff < stoneLength && !firstStone) {
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
                ssXPos = darkAreas.get(0);

                if (isRed) {
                    if (ssXPos > 60 && ssXPos < 105) {ssPosValue = 1;} // left
                    else if((ssXPos > 0 && ssXPos < 10) || (ssXPos > 105 && ssXPos < 165)) {ssPosValue = 2;} // middle
                    else if ((ssXPos > 10 && ssXPos < 60) || (ssXPos > 165 && ssXPos < 230)) {ssPosValue = 3;} // right
                } else {
                    if (ssXPos > 135 && ssXPos < 180) {ssPosValue = 1;} // left
                    else if((ssXPos > 230 && ssXPos < 240) || (ssXPos > 75 && ssXPos < 135)) {ssPosValue = 2;} // middle
                    else if ((ssXPos > 180 && ssXPos < 230) || (ssXPos > 10 && ssXPos < 75)) {ssPosValue = 3;} // right
                }

            } else log("Cannot Determine SkyStone Position :-(");
        } else log("Quarry Row Not Detected :-(");

        log("Stone Length: " + stoneLength);
        return ssPosValue;
    }

    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }

    private void log(String message) {Log.w("opencv-main", message);}

    public double getPosition() {return ssPos;}

    public double getNumberOfStones() {return curStoneCount;}

    public double getSSPosX() {return ssXPos;}

    public void setActive(boolean active) {this.active = active;}

    public void isAllianceRed(boolean isRed) {this.isRed = isRed;}
}