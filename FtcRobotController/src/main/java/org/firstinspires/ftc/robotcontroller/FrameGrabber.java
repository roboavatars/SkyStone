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
    private int saveCount = 0;
    private int getCount = 0;
    private boolean preview;

    private ArrayList<Mat> inputFrameList = new ArrayList<>();
    private boolean[] frameReadyList = {false, false, false, false, false};

    public FrameGrabber(boolean preview) {
        this.preview = preview;
    }

    @Override public void onCameraViewStarted(int width, int height) {
        log("camera started");
    }

    @Override public void onCameraViewStopped() {
        log("camera stopped");
    }

    // remember to wait a few seconds for camera aperture to open, otherwise image will be black :-)
    @Override public Mat onCameraFrame(CvCameraViewFrame cameraFrame) {
        Mat inputFrame = cameraFrame.rgba(); //always have 1 frame buffered
        if (!preview && saveCount < 5) {
            //Imgcodecs.imwrite(outputPath + saveCount + ".jpg", inputFrame);
            Mat resizedFrame = new Mat();
            Imgproc.resize(inputFrame, resizedFrame, new Size(300, 225));

            inputFrameList.add(resizedFrame);
            log("Frame " + saveCount + " Ready");
            frameReadyList[saveCount] = true;
            saveCount++;

            //try { Thread.sleep(10); } catch (InterruptedException ex) {log("you dared to interrupt me!!!!");}
        }

        // Rotate inputFrame for camera preview
        if (preview) {
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
