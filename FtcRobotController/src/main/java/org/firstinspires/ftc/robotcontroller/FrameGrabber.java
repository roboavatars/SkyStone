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

import java.util.ArrayList;

import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;
import static org.opencv.imgproc.Imgproc.warpAffine;

@SuppressLint("SdCardPath")
public class FrameGrabber implements CvCameraViewListener2 {

    private final String outputPath = "/sdcard/FIRST/openCV/cameraFrames/input"; //.jpg";
    private int saveCount = 1;
    private int getCount = 0;
    private boolean preview;

    private ArrayList<Mat> inputFrameList = new ArrayList<>();
    private boolean[] frameReadyList = {false, false, false, false, false};

    public FrameGrabber(boolean preview) {
        this.preview = preview;
    }

    @Override public void onCameraViewStarted(int width, int height) {
        log("camera started");
        inputFrameList.add(new Mat()); // placeholder
    }

    @Override public void onCameraViewStopped() {
        log("camera stopped");
    }

    // remember to wait a few seconds for camera aperture to open, otherwise image will be black :-)
    // also, it may hang the program and crash the app with SocketTimeoutException
    @Override public Mat onCameraFrame(CvCameraViewFrame cameraFrame) {
        Mat inputFrame = cameraFrame.rgba();
        Mat resizedFrame = new Mat();
        Imgproc.resize(inputFrame, resizedFrame, new Size(240, 180));

        if (preview) {
            inputFrameList.set(0, resizedFrame);
            frameReadyList[0] = true;

            // Rotate inputFrame for camera preview
            Point rawCenter = new Point(inputFrame.cols() / 2.0, inputFrame.rows() / 2.0);
            Mat rotationMatrix = getRotationMatrix2D(rawCenter, -90, 1.35);
            warpAffine(inputFrame, inputFrame, rotationMatrix, inputFrame.size());

            /*Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_BGR2HSV);
            List<Mat> hsvTypes = new ArrayList<>(3);
            Core.split(inputFrame, hsvTypes);
            Mat debug = new Mat();
            Core.inRange(hsvTypes.get(1), new Scalar(190, 120, 0), new Scalar(255, 150, 10), debug);
            return debug;*/
        }
        else if (saveCount < 5) { // !preview && saveCount < 5
            //Imgcodecs.imwrite(outputPath + saveCount + ".jpg", inputFrame);

            inputFrameList.add(resizedFrame);
            log("Frame " + saveCount + " Ready");
            frameReadyList[saveCount] = true;
            saveCount++;

            try {Thread.sleep(5);} catch (InterruptedException ex) {log("interrupted !!!");}
        }
        return inputFrame;
    }

    public Mat getNextMat() {
        Mat returnFrame = inputFrameList.get(getCount);
        log("Frame " + getCount + " Requested");
        getCount++;
        return returnFrame;
    }

    public boolean isNextFrameReady() {
        return frameReadyList[getCount];
    }

    public void setPreview(boolean preview) {
        this.preview = preview;
    }

    private void log(String message) {
        Log.w("opencv-grabber", message);
    }
}
