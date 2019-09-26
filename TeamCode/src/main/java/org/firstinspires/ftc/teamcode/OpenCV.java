package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private final String series = "A";
    private final String basePath = "/sdcard/FIRST/openCV/";
    private final String inputPath = basePath + "/cameraFrames/input.jpg";
    private final String hsvPath = basePath + "hsv" + series + ".jpg";
    private final String satNewPath = basePath + "saturationFiltered" + series + ".jpg";
    private final String horViewPath = basePath + "horizontalAvg" + series + ".jpg";
    private final String verImagePath = basePath + "verticalAvg" + series + ".jpg";

    @Override public void runOpMode() {

        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "");
        telemetry2("Using Image Series " + series, "");
        FtcRobotControllerActivity.enableCameraView();
        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber;

        while (!isStopRequested()) {
            if (frameGrabber.isImageReady()) {
                //String inputPath = basePath + "quarryRow" + series + ".jpg";
                Mat input = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                Imgproc.resize(input, input, new Size(640,480));
                telemetry2("1", "Image Loaded");

                Mat HSV = new Mat();
                Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

                Imgcodecs.imwrite(hsvPath, HSV);
                telemetry2("2", "HSV Image Saved");

                List<Mat> hsvTypes = new ArrayList<>(3);
                Core.split(HSV, hsvTypes);
                Mat saturationUnfiltered = hsvTypes.get(1);
                telemetry2("3", "Saturation Converted");

                Mat satFiltered = new Mat();
                Scalar minValue = new Scalar(180, 180, 0);
                Scalar maxValue = new Scalar(250, 250, 10);
                Core.inRange(saturationUnfiltered, minValue, maxValue, satFiltered);
                Imgcodecs.imwrite(satNewPath, satFiltered);
                telemetry2("4", "Saturation Filtered and Saved");

                double horSum;
                ArrayList<Double> horList = new ArrayList<>();
                Mat horView = new Mat(satFiltered.rows(), 1, CvType.CV_8UC1);
                for (int row = 0; row < satFiltered.rows(); row++) {
                    horSum = 0;
                    for (int col = 0; col < satFiltered.cols(); col++) {
                        horSum += satFiltered.get(row, col)[0];
                    }
                    horList.add(horSum);
                    horView.row(row).setTo(new Scalar(horSum / satFiltered.rows()));
                }
                Imgcodecs.imwrite(horViewPath, horView);
                log("Horizontal: " + horList.toString());
                telemetry2("5", "Horizontal Image Data Saved");

                Mat SCropped = new Mat();
                for (int c = 0; c < horList.size(); c++) {
                    double rowIntensity = horList.get(c);
                    if (rowIntensity > 10000) {
                        SCropped.push_back(satFiltered.row(c));
                    }
                }
                telemetry2("6", "Saturation Image Cropped");

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
                Imgcodecs.imwrite(verImagePath, verImage);
                log("Vertical: " + verList.toString());
                telemetry2("7", "Vertical Image Data Saved");

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

                telemetry2("8", "Image Analyzed");
                telemetry2("Done", "");

                //sleep(10000);
                FtcRobotControllerActivity.disableCameraView();
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
