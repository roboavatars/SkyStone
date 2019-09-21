package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="OpenCV HSV Test")
public class OpenCVTest2 extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final String VUFORIA_KEY = key.key;
    private VuforiaLocalizer vuforia = null;

    AppUtil appUtil = AppUtil.getInstance();
    BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override public void runOpMode() {

        int count = 0;
        telemetry.addData("Initializing OpenCV", "");
        telemetry.update();
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        this.vuforia.setFrameQueueCapacity(1);

        while (!isStopRequested()) {

                // Get and Save Frames
                VuforiaLocalizer.CloseableFrame frame;
                Imgcodecs imageCodecs = new Imgcodecs();
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
                            Mat INPUT = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
                            Utils.bitmapToMat(bm, INPUT);

                            //String input = "/sdcard/FIRST/vuforiaTesting/input.jpg";
                            //Mat INPUT = Imgcodecs.imread(input, Imgcodecs.IMREAD_COLOR);
                            telemetry.addData("1", "Image Loaded");
                            telemetry.update();

                            Mat HSV = new Mat();
                            Imgproc.cvtColor(INPUT, HSV, Imgproc.COLOR_BGR2HSV);
                            telemetry.addData("2", "Image Converted");
                            telemetry.update();

                            List<Mat> hsvTypes = new ArrayList<Mat>(3);
                            Core.split(HSV, hsvTypes);
                            //Mat HSVH = hsvTypes.get(0);
                            Mat HSVS = hsvTypes.get(1);
                            //Mat HSVV = hsvTypes.get(2);
                            telemetry.addData("3", "Image Types Converted");
                            telemetry.update();

                            //String hsvh = "/sdcard/FIRST/vuforiaTesting/hhsv.jpg";
                            String hsvs = "/sdcard/FIRST/vuforiaTesting/shsv.jpg";
                            //String hsvv = "/sdcard/FIRST/vuforiaTesting/vhsv.jpg";
                            //imageCodecs.imwrite(hsvh, HSVH);
                            imageCodecs.imwrite(hsvs, HSVS);
                            //imageCodecs.imwrite(hsvv, HSVV);
                            telemetry.addData("4", "Image Types Saved");
                            telemetry.update();

                            //Mat H = new Mat();
                            Mat S = new Mat();
                            //Mat V = new Mat();
                            /*Core.inRange(HSVH, new Scalar(50,10,10), new Scalar(65,100,100), H);
                            Core.inRange(HSVS, new Scalar(50,10,10), new Scalar(65,100,100), S);
                            Core.inRange(HSVV, new Scalar(50,10,10), new Scalar(65,100,100), V);*/
                            //Core.inRange(HSVH, new Scalar(180,180,0), new Scalar(255,255,10), H);
                            Core.inRange(HSVS, new Scalar(180,180,0), new Scalar(255,255,10), S);
                            //Core.inRange(HSVV, new Scalar(180,180,0), new Scalar(255,255,10), V);
                            //String HNew = "/sdcard/FIRST/vuforiaTesting/hsvh.jpg";
                            String SNew = "/sdcard/FIRST/vuforiaTesting/hsvs.jpg";
                            //String VNew = "/sdcard/FIRST/vuforiaTesting/hsvv.jpg";
                            //imageCodecs.imwrite(HNew, H);
                            imageCodecs.imwrite(SNew, S);
                            //imageCodecs.imwrite(VNew, V);
                            telemetry.addData("5", "Image Types Filtered and Saved");
                            telemetry.update();

                            String output = "/sdcard/FIRST/vuforiaTesting/output.jpg";
                            imageCodecs.imwrite(output, HSV);
                            telemetry.addData("6", "Image Saved");
                            telemetry.update();

                            count++;
                        }
                        break;
                    }
                }
                frame.close();
        }
    }
}
