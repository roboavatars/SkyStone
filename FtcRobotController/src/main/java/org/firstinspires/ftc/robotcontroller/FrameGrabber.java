package org.firstinspires.ftc.robotcontroller;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

@SuppressLint("SdCardPath")
public class FrameGrabber implements CvCameraViewListener2 {

    private boolean imageReady = false;
    private boolean saveImg = true;
    private int count;
    private final String output = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.w("opencv", "camera start");
    }

    @Override
    public void onCameraViewStopped() {

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

        return frame;
    }

    public boolean isImageReady() {
        return imageReady;
    }
}
