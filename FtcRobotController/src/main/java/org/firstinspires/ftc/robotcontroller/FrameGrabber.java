package org.firstinspires.ftc.robotcontroller;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;

import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;
import static org.opencv.imgproc.Imgproc.warpAffine;

@SuppressLint("SdCardPath")
public class FrameGrabber implements CvCameraViewListener2 {

    private final String outputPath = "/sdcard/FIRST/cameraFrames/input";
    private int saveCount = 1;
    private boolean preview;

    private Mat curMat;

    public FrameGrabber(boolean preview) {
        this.preview = preview;
    }

    @Override public void onCameraViewStarted(int width, int height) {
        log("camera started");
        File dir = new File("/sdcard/FIRST/cameraFrames/");
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {new File(dir, child).delete();}
        }
    }

    @Override public void onCameraViewStopped() {
        log("camera stopped");
    }

    // remember to wait a few seconds for camera aperture to open, otherwise image will be black :-)
    // also, it may hang the program and crash the app
    @Override public Mat onCameraFrame(CvCameraViewFrame cameraFrame) {
        Mat inputFrame = cameraFrame.rgba();
        Mat resizedFrame = new Mat();
        Imgproc.resize(inputFrame, resizedFrame, new Size(240, 180));
        curMat = resizedFrame;

        /*if (preview) {
            // Rotate inputFrame for camera preview
            Point rawCenter = new Point(inputFrame.cols() / 2.0, inputFrame.rows() / 2.0);
            Mat rotationMatrix = getRotationMatrix2D(rawCenter, -90, 1.35);
            warpAffine(inputFrame, inputFrame, rotationMatrix, inputFrame.size());
        }*/

        return inputFrame;
    }

    public Mat getNextMat() {
        //if (saveCount<200) Imgcodecs.imwrite(outputPath + saveCount + ".jpg", curMat); saveCount++;
        return curMat;
    }

    public void setPreview(boolean preview) {this.preview = preview;}

    private void log(String message) {Log.w("opencv-grabber", message);}
}
