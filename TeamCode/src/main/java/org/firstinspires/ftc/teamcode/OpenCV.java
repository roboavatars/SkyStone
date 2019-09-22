package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.opencv.core.CvType.CV_8UC1;

@Autonomous(name="OpenCV HSV Test")
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

    @Override public void runOpMode() {

        telemetry.addData("Initializing OpenCV", "");
        telemetry.update();
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        String basePath = "/sdcard/FIRST/vuforiaTesting/";
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
        this.vuforia.setFrameQueueCapacity(1);

        while (!isStopRequested()) {

            VuforiaLocalizer.CloseableFrame frame;
            try {
                frame = this.vuforia.getFrameQueue().take();
            } catch (InterruptedException ex) {
                telemetry.addData("Error", "");
                telemetry.update();
                break;
            }

            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    if (rgb != null) {
                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(rgb.getPixels());
                        //Mat INPUT = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
                        //Utils.bitmapToMat(bm, INPUT);

                        String inputPath = basePath + "row.jpg";
                        Mat INPUT = Imgcodecs.imread(inputPath, Imgcodecs.IMREAD_COLOR);
                        telemetry.addData("1", "Image Loaded");
                        telemetry.update();

                        Mat HSV = new Mat();
                        Imgproc.cvtColor(INPUT, HSV, Imgproc.COLOR_BGR2HSV);
                        String output = basePath + "hsv.jpg";
                        Imgcodecs.imwrite(output, HSV);
                        telemetry.addData("2", "Image HSV Saved");
                        telemetry.update();

                        List<Mat> hsvTypes = new ArrayList<Mat>(3);
                        Core.split(HSV, hsvTypes);
                        //Mat hue = hsvTypes.get(0);
                        Mat saturation = hsvTypes.get(1);
                        //Mat value = hsvTypes.get(2);
                        telemetry.addData("3", "Image Types Converted");
                        telemetry.update();

                        //String huePath = basePath + "hue.jpg";
                        //String saturationPath = basePath + "saturation.jpg";
                        //String valuePath = basePath + "value.jpg";
                        //Imgcodecs.imwrite(huePath, hue);
                        //Imgcodecs.imwrite(saturationPath, saturation);
                        //Imgcodecs.imwrite(valuePath, value);
                        //telemetry.addData("4", "Image Types Saved");
                        //telemetry.update();

                        //Mat H = new Mat();
                        Mat S = new Mat();
                        //Mat V = new Mat();
                        /*Core.inRange(hue, new Scalar(50,10,10), new Scalar(65,100,100), H);
                        Core.inRange(saturation, new Scalar(50,10,10), new Scalar(65,100,100), S);
                        Core.inRange(value, new Scalar(50,10,10), new Scalar(65,100,100), V);*/
                        //Core.inRange(hue, new Scalar(180,180,0), new Scalar(255,255,10), H);
                        Core.inRange(saturation, new Scalar(180,180,0), new Scalar(255,255,10), S);
                        //Core.inRange(value, new Scalar(180,180,0), new Scalar(255,255,10), V);

                        //String hueNew = basePath + "hue.jpg";
                        String satNew = basePath + "saturation.jpg";
                        //String valNew = basePath + "value.jpg";
                        //Imgcodecs.imwrite(hueNew, H);
                        Imgcodecs.imwrite(satNew, S);
                        //Imgcodecs.imwrite(valNew, V);
                        telemetry.addData("4", "Image Types Filtered and Saved");
                        telemetry.update();

                        double sum;
                        ArrayList sumList = new ArrayList();
                        Mat newImage = new Mat(S.rows(), S.cols(), CV_8UC1);
                        for (int col = 0; col < S.cols(); col++) {
                            sum = 0;
                            for (int row = 0; row < S.rows(); row++) {
                                sum += S.get(row,col)[0];
                            }
                            sum *= 2;
                            sumList.add(sum);
                            newImage.col(col).setTo(new Scalar(sum / S.rows()));
                        }
                        System.out.println(newImage.row(0));
                        String newImageName = basePath + "verticalAvg.jpg";
                        Imgcodecs.imwrite(newImageName, newImage);
                        System.out.println(sumList);
                        telemetry.addData("5", "Image Data Vertical Saved");
                        telemetry.update();

                        double sum2;
                        ArrayList sumList2 = new ArrayList();
                        Mat newImage2 = new Mat(S.rows(), S.cols(), CV_8UC1);
                        for (int row = 0; row < S.rows(); row++) {
                            sum2 = 0;
                            for (int col = 0; col < S.cols(); col++) {
                                sum2 += S.get(row,col)[0];
                            }
                            sumList2.add(sum2);
                            newImage2.row(row).setTo(new Scalar(sum2 / S.rows()));
                        }
                        System.out.println(newImage2.col(0));
                        String newImageName2 = basePath + "horizontalAvg.jpg";
                        Imgcodecs.imwrite(newImageName2, newImage2);
                        System.out.println(sumList2);
                        telemetry.addData("6", "Image Data Horizontal Saved");
                        telemetry.update();

                        sleep(10000);
                    }
                    break;
                }
            }
            frame.close();
        }
    }
}
