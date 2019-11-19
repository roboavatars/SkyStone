package org.firstinspires.ftc.robotcontroller;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;

/**
 * OpenCV Camera Interface
 * <p>Used to get camera frames
 */

@SuppressLint("SdCardPath")
public class FrameGrabber implements CvCameraViewListener2 {

    private final String outputPath = "/sdcard/FIRST/cameraFrames/input";
    private int saveCount = 1;
    private Mat curMat;

    public FrameGrabber() {}

    @Override public void onCameraViewStarted(int width, int height) {
        log("camera started");

        // clear folder of images
        File dir = new File("/sdcard/FIRST/cameraFrames/");
        String[] children = dir.list();
        if (children != null) {for (String child : children) {new File(dir, child).delete();}}
    }

    @Override public void onCameraViewStopped() {
        log("camera stopped");
    }

    @Override public Mat onCameraFrame(CvCameraViewFrame cameraFrame) {
        Mat inputFrame = cameraFrame.rgba();
        Mat resizedFrame = new Mat();
        Imgproc.resize(inputFrame, resizedFrame, new Size(240, 180));
        curMat = resizedFrame;
        return inputFrame;
    }
    
    /**
     * Returns the most recent camera frame (downsized to 240x180)
     * @return Most recent camera frame
     */
    public Mat getNextMat() {
        //if (saveCount < 100) Imgcodecs.imwrite(outputPath + saveCount + ".jpg", curMat); saveCount++;
        return curMat;
    }

    private void log(String message) {Log.w("opencv-grabber", message);}
}
