package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

@SuppressWarnings("FieldCanBeLocal") @SuppressLint("SdCardPath")
public class Logger {

    private static File robotDataLog;
    private static String basePath = "/sdcard/FIRST/robotLogs/RobotData";
    //private static String basePath = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Tests/testLogs/TestLog";
    private static FileWriter fileWriter;
    private static BufferedReader bufferedReader;

    public void startLogging() {
        try {
            robotDataLog = new File(getLogName(true));
            fileWriter = new FileWriter(robotDataLog);
            fileWriter.write("Timestamp,SinceStart,X,Y,Theta,VelocityX,VelocityY,VelocityTheta,StoneInRobot,StoneClamped,TryingToDeposit,ArmIsHome,ArmIsDown,ArmIsOut\n");
        } catch (Exception e) {e.printStackTrace();}
    }

    public static String getLogName(boolean fileWrite) {
        int logNum = 1;
        while (true) {
            File currentFile = new File(basePath + logNum + ".csv");
            if (!currentFile.exists()) break;
            logNum++;
        }
        if (fileWrite) return basePath + logNum + ".csv";
        else return basePath + (logNum-1) + ".csv";
    }

    public void logData(double timeSinceSt, double x, double y, double theta, double velocityx, double velocityy, double velocitytheta, boolean stoneInRobot, boolean stoneClamped, boolean tryingToDeposit, boolean armIsHome, boolean armIsDown, boolean armIsOut) {
        @SuppressLint("SimpleDateFormat")
        SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss.SSS"); Date d = new Date();
        try {
            fileWriter.write(df.format(d)+","+timeSinceSt+","+x+","+y+","+theta+","+velocityx+","+velocityy+","+velocitytheta+","+stoneInRobot+","+stoneClamped+","+tryingToDeposit+","+armIsHome+","+armIsDown+","+armIsOut+"\n");
        } catch (Exception ex) {ex.printStackTrace();}
    }

    public void flush() {
        try {fileWriter.flush();}
        catch (Exception e) {e.printStackTrace();}
    }

    public void stopLogging() {
        try {fileWriter.close();}
        catch (Exception e) {e.printStackTrace();}
    }

    public static double[] readPos() {
        String curLine; int lineNum = 0;
        double[] robotPos = new double[3];

        try {
            bufferedReader = new BufferedReader(new FileReader(getLogName(false)));
            while ((curLine = bufferedReader.readLine()) != null) {

                if (lineNum != 0) {
                    String[] data = curLine.split(","); //System.out.println(Arrays.toString(data));
                    robotPos[0] = Double.parseDouble(data[2]);
                    robotPos[1] = Double.parseDouble(data[3]);
                    robotPos[2] = Double.parseDouble(data[4]);
                    //System.out.println(Arrays.toString(robotPos);
                }
                lineNum++;
            }
            bufferedReader.close();
        } catch (Exception ex) {
            robotPos[0] = 0; robotPos[1] = 0; robotPos[2] = 0;
            System.out.println("read error, using default values :-(");
            ex.printStackTrace();
        }

        return robotPos;
    }

//    public static void main(String[] args) {
//        double[] initialPosition = Logger.readPos();
//        System.out.println("Starting Position: " + Arrays.toString(initialPosition));
//
//        Logger logger = new Logger();
//        logger.startLogging();
//        logger.logData(0,-1,-1,-1,-100,-100,-100,false,false,false,false,false);
//        logger.flush();
//        logger.stopLogging();
//    }
}
