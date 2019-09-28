package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.FrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
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
@TeleOp(name="OpenCV Horizontal Croppping Test")
public class OpenCVHorizontalCrop extends LinearOpMode {

    private final String series = "Ansh";
    private final String basePath = "/sdcard/FIRST/openCV/ansh/";
    //private final String inputPath = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";
    private final String inputPath = basePath + "quarryRowC.jpg";
    private final String satNew = basePath + "saturationFiltered" + series + ".jpg";
    private final String open = basePath + "opened" + series + ".jpg";
    private final String horViewName = basePath + "horizontalAvg" + series + ".jpg";
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
                Mat input = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                Imgproc.resize(input, input, new Size(400,300));

                // convert hsv
                Mat HSV = new Mat();
                Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);

                List<Mat> hsvTypes = new ArrayList<>(3);
                Core.split(HSV, hsvTypes);
                Mat saturationUnfiltered = hsvTypes.get(1);

                Mat S = new Mat();
                Core.inRange(saturationUnfiltered, new Scalar(180,180,0), new Scalar(255,255,10), S);
                Imgcodecs.imwrite(satNew, S);
                telemetry2("1", "Saturation Saved");

                Mat opened = new Mat();
                Imgproc.morphologyEx(S, opened, Imgproc.MORPH_OPEN, new Mat());
                Imgcodecs.imwrite(open, opened);
                telemetry2("2", "Opened Saved");

                telemetry2("Time", time.milliseconds() + "");
                sleep(1000);

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
                telemetry2("5", "Horizontal Image Data Saved");

                telemetry2("Time", time.milliseconds() - 1000 + "");
                sleep(1000);

                Mat SCropped = new Mat();
                for (int c = 0; c < horList.size(); c++) {
                    double rowIntensity = horList.get(c);
                    if (rowIntensity > 10000) {
                        SCropped.push_back(opened.row(c));
                    }
                }
                Imgcodecs.imwrite(croppedName, SCropped);
                telemetry2("7", "Saturation Image Cropped");

                telemetry2("Time", time.milliseconds() - 2000 + "");
                sleep(1000);

                if (usingCamera) FtcRobotControllerActivity.disableCameraView();
                stop();
            }
        }
    }

    public void telemetry2(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public void log(String message) {
        Log.w("opencv", message);
    }
}
