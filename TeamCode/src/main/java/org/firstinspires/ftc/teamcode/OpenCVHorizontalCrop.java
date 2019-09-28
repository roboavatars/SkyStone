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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="OpenCV Test Ansh")
public class OpenCVHorizontalCrop extends LinearOpMode {

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

    private String series = "Ansh";

    @Override public void runOpMode() {

        telemetry2("Initializing OpenCV v" + OpenCVLoader.OPENCV_VERSION, "");
        telemetry2("Using Image Series " + series, "");
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        String basePath = "/sdcard/FIRST/openCV/";

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
                        /*Mat INPUT = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
                        Utils.bitmapToMat(bm, INPUT);
                        Imgproc.resize(INPUT, INPUT, new Size(640,360));

                        String inputPath = basePath + "input" + series + ".jpg";
                        Imgcodecs.imwrite(inputPath, newINPUT);
                        telemetry2("1", "Image Input Loaded");*/
                        //String inputPath = basePath + "quarryRow.jpg";
                        String inputPath = "/sdcard/DCIM/Camera/IMG_20190922_180721469.jpg";
                        Mat INPUT = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                        Imgproc.resize(INPUT, INPUT, new Size(640,360));
                        Imgcodecs.imwrite(inputPath, INPUT);
                        telemetry2("1", "Image Input Loaded");

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
                        Mat horView = new Mat(S.rows(), 100, CvType.CV_8UC1);
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

                        Mat SCropped = new Mat();
                        for (int c = 0; c < horList.size(); c++) {
                            double rowIntensity = horList.get(c);
                            if (rowIntensity > 10000) {
                                SCropped.push_back(S.row(c));
                            }
                        }
                        String croppedName = basePath + "croppedImage" + series + ".jpg";
                        Imgcodecs.imwrite(croppedName, SCropped);
                        telemetry2("7", "Saturation Image Cropped");

                        //sleep(10000);
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
