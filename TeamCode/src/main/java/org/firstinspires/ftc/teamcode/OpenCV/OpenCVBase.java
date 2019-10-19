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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint("SdCardPath")
public class OpenCVBase extends Thread {
    private LinearOpMode op;
    OpenCVBase(LinearOpMode opMode) {op = opMode;}

    // File Paths
    private final String series = "A";
    private final String basePath = "/sdcard/FIRST/openCV/";
    private final String satNewPath = basePath + "saturationFiltered" + series + ".jpg";
    private final String openClosePath = basePath + "openClose" + series + ".jpg";
    private final String croppedName = basePath + "croppedImage" + series;
    private final String verViewName = basePath + "verticalAvg" + series;
    private final String testPath = basePath + "/testFiles/test";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------

    private ElapsedTime timer = new ElapsedTime();

    private final int binaryThreshold = 250;
    private int count = 1;

    private enum skyStonePosition {Not_Found, First_Left, Second_Left, Third_Left}

    private boolean ready = false;
    private int dataSum = 0;
    private int dataCount = 0;
    private int verdict = -1;

    void initializeCamera() {
        telemetry2("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) {FtcRobotControllerActivity.showCameraPreview();}
    }

    @Override
    public void run() {
        timer.reset();
        setName("OpenCV");

        if (usingCamera) {
            FtcRobotControllerActivity.hidePreview();
            frameGrabber = FtcRobotControllerActivity.frameGrabber;
            logTime("Init Time");

            for (int frame = 0; frame < 5; frame++) {

                log("Frame " + frame + " ----------------------------------------");
                while (!frameGrabber.isNextFrameReady()) {log("Waiting for frame " + frame);}
                logTime("Input Ready Time");
                skyStonePosition curPosition = detectSkyStone(frameGrabber.getNextMat());

                dataSum += curPosition.ordinal();
                if (curPosition.ordinal() != 0) {dataCount++;}

                log("Left SkyStone Position: " + curPosition);
            }
            ready = true;
            if (dataCount == 0) {verdict = 0;}
            else {verdict = Math.round(dataSum/dataCount);}
            log("Done");

        } else {
            for (; count <= 4; count++) {
                detectSkyStone(Imgcodecs.imread(testPath + count + ".jpg", Imgcodecs.IMREAD_COLOR));}
        }

        //sleep(2000);
        if (usingCamera) FtcRobotControllerActivity.disableCameraView();
        log(" ");
    }

    private skyStonePosition detectSkyStone (Mat input) {
        skyStonePosition leftSkyStonePos; // 3, 3, 1, 2

        // Convert to HSV (Saturation)
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
        List<Mat> hsvTypes = new ArrayList<>(3);
        Core.split(HSV, hsvTypes);
        Mat satUnfiltered = hsvTypes.get(1);

        // Filter Saturation Image
        Mat satFiltered = new Mat();
        Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 150, 10), satFiltered);
        //Imgcodecs.imwrite(satNewPath, satFiltered);

        Mat openClose = new Mat();
        Imgproc.morphologyEx(satFiltered, openClose, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(openClose, openClose, Imgproc.MORPH_CLOSE, new Mat());
        //Imgcodecs.imwrite(openClosePath, openClose);
        logTime("Filter Time");

        // Crop Image to where stone row is
        double horAvg;
        Mat SCropped = new Mat();
        for (int row = 0; row < openClose.rows(); row++) {
            horAvg = Core.mean(openClose.row(row)).val[0];
            if (horAvg > 150) {SCropped.push_back(openClose.row(row));}
        }

        if (!(SCropped.cols() == 0)) {
            log("Quarry Row Detected :-)");
            //Imgcodecs.imwrite(croppedName + ".jpg", SCropped);
            //Imgcodecs.imwrite(croppedName + count + ".jpg", SCropped);
            logTime("Crop Time");

            double verAvg;
            Mat verImage = new Mat(1, SCropped.cols(), CvType.CV_8UC1);
            for (int col = 0; col < SCropped.cols(); col++) {
                verAvg = Core.mean(openClose.col(col)).val[0] * 10;
                if (verAvg <= 225) {verAvg = 0;}
                else {verAvg = binaryThreshold;}
                verImage.col(col).setTo(new Scalar(verAvg));
            }
            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());
            //Imgcodecs.imwrite(verViewName + ".jpg", verImage);
            //Imgcodecs.imwrite(verViewName + count + ".jpg", verImage);

            /*String verCols = "";
            for (int col = 0; col < verImage.cols(); col++) {
                verCols += (new Scalar(verImage.get(0, col)).val[0]) + ", ";
            }
            log("Vertical: " + verCols);*/
            logTime("Vertical Time");

            ArrayList<Integer> /*darkCols = new ArrayList<>(),*/ darkAreas = new ArrayList<>();
            double prevIntensity = 0;
            int columnsBack = 15;
            for (int c = 0; c < verImage.cols(); c++) {
                double curIntensity = new Scalar(verImage.get(0, c)).val[0];

                if (curIntensity == 0) {
                    //darkCols.add(c);

                    double intensityDiff;
                    if (c < columnsBack) intensityDiff = Math.abs(curIntensity - prevIntensity);
                    else intensityDiff = Math.abs(curIntensity
                            - new Scalar(verImage.get(0, c - columnsBack)).val[0]);

                    if (intensityDiff == binaryThreshold) {
                        darkAreas.add(c);
                    }
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
                if (areaDiff < 75 && !firstStone) {
                    darkAreas.remove(a);
                    a--;
                }
                firstStone = false;

                prevArea = curArea;
            }
            //log("Rev1 Dark Areas: " + darkAreas);

            if (darkAreas.size() == 3) darkAreas.remove(darkAreas.size() - 1);
            //log("Rev2 Dark Areas: " + darkAreas);
            log("SkyStones Detected: " + darkAreas.size());

            if (!(darkAreas.size() == 0)) {
                if (darkAreas.get(0) <= 40) leftSkyStonePos = skyStonePosition.First_Left;
                else if (darkAreas.get(0) > 40 && darkAreas.get(0) <= 80) leftSkyStonePos = skyStonePosition.Second_Left;
                else leftSkyStonePos = skyStonePosition.Third_Left;
                log("Location: " + leftSkyStonePos);
            } else {
                leftSkyStonePos = skyStonePosition.Not_Found;
                log("Cannot Determine Left SkyStone Position :-(");
            }
            logTime("Analysis Time");
            telemetry2("End Info", darkAreas.size() + " " + leftSkyStonePos + " " + timer.milliseconds());
        } else {
            leftSkyStonePos = skyStonePosition.Not_Found;
            log("Quarry Row Not Detected :-(");
            telemetry2("End Info", "Row Not Detected " + timer.milliseconds());
        }
        return leftSkyStonePos;
    }

    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }

    private void log(String message) {Log.w("opencv-main", message);}

    private void logTime(String message) {log(message + ": " + timer.milliseconds());}

    boolean isReady() {return ready;}

    int getPosition() {return verdict;}

    double getFps() {return 5000.0 / timer.milliseconds();}
}