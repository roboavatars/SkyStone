package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="OpenCV Test")
public class OpenCV extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final String VUFORIA_KEY = key.key;
    private VuforiaLocalizer vuforia = null;

    private AppUtil appUtil = AppUtil.getInstance();
    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    private String series = "B";

    @Override public void runOpMode() {

        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "");
        telemetry2("Using Image Series " + series, "");
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        String basePath = "/sdcard/FIRST/openCV/";
        /*File dir = new File(basePath);
        String[] children = dir.list();
        for (String child : children) {
            new File(dir, child).delete();
        }*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        while (!isStopRequested()) {

            VuforiaLocalizer.CloseableFrame frame;
            try {
                frame = vuforia.getFrameQueue().take();
            } catch (InterruptedException ex) {
                telemetry2("Frame Error", "");
                break;
            }

            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    if (rgb != null) {
                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(rgb.getPixels());
                        Mat INPUT = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
                        Utils.bitmapToMat(bm, INPUT);

                        //String inputPath = basePath + "quarryRow" + series + ".jpg";
                        //Mat INPUT = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                        telemetry2("1", "Image Loaded");

                        Mat HSV = new Mat();
                        Imgproc.cvtColor(INPUT, HSV, Imgproc.COLOR_BGR2HSV);
                        String output = basePath + "hsv" + series + ".jpg";
                        Imgcodecs.imwrite(output, HSV);
                        telemetry2("2", "HSV Image Saved");

                        List<Mat> hsvTypes = new ArrayList<>(3);
                        Core.split(HSV, hsvTypes);
                        Mat saturationUnfiltered = hsvTypes.get(1);
                        telemetry2("3", "Saturation Converted");

                        Mat S = new Mat();
                        Core.inRange(saturationUnfiltered, new Scalar(180,180,0), new Scalar(255,255,10), S);
                        String satNew = basePath + "saturationFiltered" + series + ".jpg";
                        Imgcodecs.imwrite(satNew, S);
                        telemetry2("4", "Saturation Filtered and Saved");

                        double horSum;
                        ArrayList<Double> horList = new ArrayList<>();
                        Mat horView = new Mat(S.rows(), 1, CvType.CV_8UC1);
                        for (int row = 0; row < S.rows(); row++) {
                            horSum = 0;
                            for (int col = 0; col < S.cols(); col++) {
                                horSum += S.get(row,col)[0];
                            }
                            horList.add(horSum);
                            horView.row(row).setTo(new Scalar(horSum / S.rows()));
                        }
                        log("Horizontal Col0: " + horView.col(0).toString());
                        String horViewName = basePath + "horizontalAvg" + series + ".jpg";
                        Imgcodecs.imwrite(horViewName, horView);
                        log("Horizontal: " + horList.toString());
                        telemetry2("5", "Horizontal Image Data Saved");

                        Mat tempMat1;
                        Mat tempMat2;
                        Mat SCropped = S.clone();
                        for (int c = 0; c < horList.size(); c++) {
                            double rowIntensity = horList.get(c);
                            if (rowIntensity < 10000) {
                                Rect beforeC = new Rect(0, 0, SCropped.cols(), c);
                                tempMat1 = new Mat(SCropped, beforeC);
                                Rect afterC = new Rect(0, 0, SCropped.cols(), SCropped.rows()-c);
                                tempMat2 = new Mat(SCropped, afterC);
                                SCropped = tempMat1;
                                SCropped.push_back(tempMat2);
                            }
                        }
                        telemetry2("6", "Saturation Image Cropped");

                        double verSum;
                        ArrayList<Double> verList = new ArrayList<>();
                        Mat verImage = new Mat(SCropped.rows(), SCropped.cols(), CvType.CV_8UC1);
                        for (int col = 0; col < SCropped.cols(); col++) {
                            verSum = 0;
                            for (int row = 0; row < SCropped.rows(); row++) {
                                verSum += SCropped.get(row,col)[0];
                            }
                            verSum *= 2;
                            verList.add(verSum);
                            verImage.col(col).setTo(new Scalar(verSum / SCropped.rows()));
                        }
                        log("Vertical Row0: " + verImage.row(0).toString());
                        String verImageName = basePath + "verticalAvg" + series + ".jpg";
                        Imgcodecs.imwrite(verImageName, verImage);
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

                        String inputPath = basePath + "quarryRow" + series + ".jpg";
                        Imgcodecs.imwrite(inputPath, INPUT);
                        telemetry2("Done", "");

                        sleep(10000);
                        stop();
                    }
                    break;
                }
            }
            frame.close();
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
