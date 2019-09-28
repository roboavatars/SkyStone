package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name="OpenCV Test")
public class OpenCV extends LinearOpMode {

    private final String series = "C";
    private final String basePath = "/sdcard/FIRST/openCV/";
    //private final String inputPath = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";
    private final String inputPath = basePath + "quarryRowC.jpg";
    private final String satNew = basePath + "saturationFiltered" + series + ".jpg";
    private final String open = basePath + "opened" + series + ".jpg";
    private final String horViewName = basePath + "horizontalAvg" + series + ".jpg";
    private final String verViewName = basePath + "verticalAvg" + series + ".jpg";
    private final String croppedName = basePath + "croppedImage" + series + ".jpg";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = false;
    private ElapsedTime time = new ElapsedTime();

    @Override public void runOpMode() {

        time.startTime();
        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "");
        telemetry2("Using Image Series " + series, "");
        if (usingCamera) {
            FtcRobotControllerActivity.enableCameraView();
            frameGrabber = FtcRobotControllerActivity.frameGrabber;
        }

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

                telemetry2("Filter Time", time.milliseconds() + "");
                sleep(1000);

                // Average Rows
                double horSum;
                ArrayList<Double> horList = new ArrayList<>();
                Mat horView = new Mat(opened.rows(), 100, CvType.CV_8UC1);
                for (int row = 0; row < opened.rows(); row++) {
                    horSum = 0;
                    for (int col = 0; col < opened.cols(); col++) {
                        horSum += opened.get(row,col)[0];
                    }
                    horList.add(horSum);
                    horView.row(row).setTo(new Scalar(horSum / opened.rows()));
                }
                log("Horizontal Col0: " + horView.col(0).toString());
                log("Horizontal: " + horList.toString());
                Imgcodecs.imwrite(horViewName, horView);
                telemetry2("3", "Horizontal Data Saved");

                telemetry2("Post-Avg Time", time.milliseconds() - 1000 + "");
                sleep(1000);

                // Crops Filtered Image
                Mat SCropped = new Mat();
                for (int c = 0; c < horList.size(); c++) {
                    double rowIntensity = horList.get(c);
                    if (rowIntensity > 40000) {
                        SCropped.push_back(opened.row(c));
                    }
                }
                Imgcodecs.imwrite(croppedName, SCropped);
                telemetry2("4", "Filtered Image Cropped");

                telemetry2("Finished Time", time.milliseconds() - 2000 + "");
                sleep(1000);

                double verSum;
                ArrayList<Double> verList = new ArrayList<>();
                Mat verImage = new Mat(SCropped.rows(), SCropped.cols(), CvType.CV_8UC1);
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

                log("Size: " + verList.size());
                String darkCols = "", darkAreas = "";
                int skyStones = 0;
                double prevIntensity = 0;
                for (int c = 0; c < verList.size(); c++) {
                    double curIntensity = verList.get(c);

                    if (curIntensity < 10000) {
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

                //sleep(10000);
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
}
