package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"FieldCanBeLocal", "StringConcatenationInLoop"})
@SuppressLint("SdCardPath")
@Autonomous(name="OpenCV Test")
public class OpenCV extends LinearOpMode {

    private final String series = "C";
    private final String basePath = "/sdcard/FIRST/openCV/";
    private final String inputPath = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";
    //private final String inputPath = basePath + "quarryRowC.jpg";
    private final String satNew = basePath + "saturationFiltered" + series + ".jpg";
    private final String open = basePath + "opened" + series + ".jpg";
    private final String horViewName = basePath + "horizontalAvg" + series + ".jpg";
    private final String verViewName = basePath + "verticalAvg" + series + ".jpg";
    private final String croppedName = basePath + "croppedImage" + series + ".jpg";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true;
    private ElapsedTime time = new ElapsedTime();

    @Override public void runOpMode() {

        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "");
        telemetry2("Using Image Series " + series, "");
        if (usingCamera) {
            FtcRobotControllerActivity.enableCameraView();
            frameGrabber = FtcRobotControllerActivity.frameGrabber;
        }

        waitForStart();
        time.startTime();

        while (!isStopRequested()) {
            if (!usingCamera || frameGrabber.isImageReady()) {
                // Input Image
                Mat input = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                Imgproc.resize(input, input, new Size(400,300));

                // Convert to HSV (Saturation) and Save
                Mat HSV = new Mat();
                Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
                List<Mat> hsvTypes = new ArrayList<>(3);
                Core.split(HSV, hsvTypes);
                Mat saturationUnfiltered = hsvTypes.get(1);
                Mat S = new Mat();
                Core.inRange(saturationUnfiltered, new Scalar(180,180,0), new Scalar(255,255,10), S);
                Imgcodecs.imwrite(satNew, S);
                telemetry2("1", "Saturation Saved");

                // Filter Saturation Image
                Mat opened = new Mat();
                Imgproc.morphologyEx(S, opened, Imgproc.MORPH_OPEN, new Mat());
                Imgproc.morphologyEx(opened, opened, Imgproc.MORPH_CLOSE, new Mat());
                Imgcodecs.imwrite(open, opened);
                telemetry2("2", "Filtered Image Saved");
                logTime("Filter Time");

                // Averages Rows
                double horAvg;
                ArrayList<Double> horList = new ArrayList<>();
                Mat horView = new Mat(opened.rows(), 1, CvType.CV_8UC1);
                for (int row = 0; row < opened.rows(); row++) {
                    horAvg = Core.mean(opened.row(row)).val[0];
                    horList.add(horAvg);
                    horView.row(row).setTo(new Scalar(horAvg));
                }
                //log("Horizontal: " + horList.toString());
                //Imgcodecs.imwrite(horViewName, horView);
                telemetry2("3", "Horizontal Data Analyzed");
                logTime("Post-Horizontal Time");

                // Crops Filtered Image
                Mat SCropped = new Mat();
                for (int c = 0; c < horList.size(); c++) {
                    if (horList.get(c) > 35000) {
                        SCropped.push_back(opened.row(c));
                    }
                }
                Imgcodecs.imwrite(croppedName, SCropped);
                telemetry2("4", "Filtered Image Cropped");
                logTime("Post-Crop Time");

                double verSum;
                ArrayList<Double> verList = new ArrayList<>();
                Mat verImage = new Mat(50, SCropped.cols(), CvType.CV_8UC1);
                for (int col = 0; col < SCropped.cols(); col++) {
                    verSum = 0;
                    for (int row = 0; row < SCropped.rows(); row++) {
                        verSum += SCropped.get(row, col)[0];
                    }
                    verSum *= 2;
                    verList.add(verSum);
                    verImage.col(col).setTo(new Scalar(verSum / SCropped.rows()));
                }
                Imgcodecs.imwrite(verViewName, verImage);
                log("Vertical: " + verList.toString());
                telemetry2("5", "Vertical Image Data Saved");
                logTime("Post-Vertical Time");

                // Algorithm
                log("Size: " + verList.size());
                String darkCols = "";
                String darkAreas = "";
                int skyStones = 0;
                double prevIntensity = 0;
                for (int c = 0; c < verList.size(); c++) {
                    double curIntensity = verList.get(c);

                    if (curIntensity < 33) {
                        darkCols += c + "(" + curIntensity + "), ";

                        double intensityDiff = Math.abs(curIntensity - prevIntensity);
                        if (intensityDiff > 20000) {
                            darkAreas += c + "(" + curIntensity + " " + intensityDiff + "), ";
                            skyStones++;
                        }
                    }
                    prevIntensity = curIntensity;
                }
                log("Dark Columns: " + darkCols);
                log("Dark Areas: " + darkAreas);
                log("SkyStones: " + (skyStones));
                telemetry2("SkyStones", skyStones + "");
                telemetry2("6", "Image Analyzed");
                telemetry2("Done", "");
                logTime("Finish Time");
                sleep(3000);

                if (usingCamera) FtcRobotControllerActivity.disableCameraView();
                stop();
            }
        }
    }

    private void telemetry2(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    private void log(String message) {
        Log.w("opencv", message);
    }

    private void logTime(String message) {
        log(message + ": " + time.milliseconds());
        telemetry2(message, time.milliseconds() + "");
    }
}
