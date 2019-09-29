package org.firstinspires.ftc.robotcontroller;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;

import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;
import static org.opencv.imgproc.Imgproc.warpAffine;

@SuppressLint("SdCardPath")
public class FrameGrabber implements CvCameraViewListener2 {

    private boolean imageReady = false;
    private boolean saveImg = true;
    private int count;
    private final String output = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.w("opencv", "camera started");
    }

    @Override
    public void onCameraViewStopped() {
        Log.w("opencv", "camera stopped");
    }

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        Mat frame = inputFrame.rgba();
        if (saveImg) {
            count++;
            if (count == 15) {
                Imgcodecs.imwrite(output, frame);
                Log.w("opencv", "Input Image Ready");
                saveImg = false;
                imageReady = true;
            }
        }

        // Rotate frame for camera view (does not affect saved image)
        Point rawCenter = new Point(frame.cols() / 2.0F, frame.rows() / 2.0F);
        Mat rotationMatrix = getRotationMatrix2D(rawCenter, -90, 1.0);
        warpAffine(frame, frame, rotationMatrix, frame.size());
        return frame;
    }

    public boolean isImageReady() {
        return imageReady;
    }
}
