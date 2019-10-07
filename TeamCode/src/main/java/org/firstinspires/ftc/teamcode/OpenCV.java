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

@SuppressWarnings("FieldCanBeLocal")
@SuppressLint("SdCardPath")
@Autonomous(name="z OpenCV")
public class OpenCV extends LinearOpMode {

    private final String series = "A";
    private final String basePath = "/sdcard/FIRST/openCV/";
    private final String satNewPath = basePath + "saturationFiltered" + series + ".jpg";
    private final String openClosePath = basePath + "openClose" + series + ".jpg";
    //private final String horViewName = basePath + "horizontalAvg" + series + ".jpg";
    private final String verViewName = basePath + "verticalAvg" + series;
    private final String croppedName = basePath + "croppedImage" + series;

    private Mat input;
    private final String inputPath = basePath + "/testFiles/test";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = false;

    private final int binaryThreshold = 250;

    private ElapsedTime timer = new ElapsedTime();

    @Override public void runOpMode() {

        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "Using Image Series " + series);
        if (usingCamera) FtcRobotControllerActivity.showView();

        waitForStart();
        timer.reset();

        /*if (usingCamera) {
            FtcRobotControllerActivity.enableCameraView();
            frameGrabber = FtcRobotControllerActivity.frameGrabber;
            logTime("Init Time");
            while (!frameGrabber.isImageReady()) {}
            logTime("Input Ready Time");
            input = frameGrabber.getInputMat();
        } else {
            input = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
        }*/

        for (int xyz = 1; xyz <= 5; xyz++) {
            input = Imgcodecs.imread(inputPath+xyz+".jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(300, 225));
            logTime("Input Get Time");

            // Convert to HSV (Saturation) and Save
            Mat HSV = new Mat();
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
            List<Mat> hsvTypes = new ArrayList<>(3);
            Core.split(HSV, hsvTypes);
            Mat satUnfiltered = hsvTypes.get(1);
            Mat satFiltered = new Mat();
            Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 150, 10), satFiltered);
            //Imgcodecs.imwrite(satNewPath, satFiltered);

            // Filter Saturation Image
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
                if (horAvg > 150) {
                    SCropped.push_back(openClose.row(row));
                }
            }
            Imgcodecs.imwrite(croppedName+xyz+".jpg", SCropped);
            logTime("Crop Time");

            double verAvg;
            ArrayList<Double> verList = new ArrayList<>();
            Mat verImage = new Mat(25, SCropped.cols(), CvType.CV_8UC1);
            for (int col = 0; col < SCropped.cols(); col++) {
                verAvg = Core.mean(openClose.col(col)).val[0] * 10;
                   verList.add(verAvg);
                if (verAvg <= 225) verAvg = 0;
                else verAvg = binaryThreshold;
                verImage.col(col).setTo(new Scalar(verAvg));
            }
            log("Vertical1: " + verList.toString());
            verList.clear();

            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());
            for (int col = 0; col < verImage.cols(); col++) {
                verList.add(new Scalar(verImage.get(0, col)).val[0]);
            }
            //log("Vertical2: " + verList.toString());
            logTime("Vertical Time");
            Imgcodecs.imwrite(verViewName+xyz+".jpg", verImage);

            ArrayList<Integer> darkCols = new ArrayList<>(), darkAreas = new ArrayList<>();
            double prevIntensity = 0;
            int columnsBack = 10;
            for (int c = 0; c < verList.size(); c++) {
                double curIntensity = verList.get(c);

                if (curIntensity == 0) {
                    darkCols.add(c);

                    double intensityDiff;
                    if (c < columnsBack) intensityDiff = Math.abs(curIntensity - prevIntensity);
                    else intensityDiff = Math.abs(curIntensity - verList.get(c - columnsBack));

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
            log("Updated Dark Areas: " + darkAreas);
            log("SkyStones: " + darkAreas.size());

            logTime("Analysis Time");
            telemetry2("SkyStones", darkAreas.size() + "");
        }

        if (usingCamera) FtcRobotControllerActivity.disableCameraView();
        log(" ");
    }

    private void telemetry2(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    private void log(String message) {
        Log.w("opencv", message);
    }

    private void logTime(String message) {
        //log(message + ": " + timer.milliseconds());
    }
}
