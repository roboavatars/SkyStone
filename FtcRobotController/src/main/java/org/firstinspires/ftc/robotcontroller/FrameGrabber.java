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
    private final String outputPath = "/sdcard/FIRST/openCV/cameraFrames/input.jpg";
    private boolean saveImg = true;
    private int count;
    private boolean preview;
    private Mat inputMat;

    public FrameGrabber(boolean preview) {
        this.preview = preview;
    }

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
        if (saveImg && !preview) {
            //count++;
            //if (count == 1) {
                inputMat = frame;
                //Imgcodecs.imwrite(outputPath, frame);
                Log.w("opencv", "Input Image Ready");
                saveImg = false;
                imageReady = true;
            //}
        }

        // Rotate frame for camera preview
        if (preview) {
            Point rawCenter = new Point(frame.cols() / 2.0, frame.rows() / 2.0);
            Mat rotationMatrix = getRotationMatrix2D(rawCenter, -90, 1.0);
            warpAffine(frame, frame, rotationMatrix, frame.size());
        }
        return frame;
    }

    public void setPreview(boolean preview) {
        this.preview = preview;
    }

    public Mat getInputMat() {
        return inputMat;
    }

    public boolean isImageReady() {
        return imageReady;
    }
}