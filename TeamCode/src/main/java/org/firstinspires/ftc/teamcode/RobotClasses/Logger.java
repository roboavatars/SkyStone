package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

@SuppressWarnings("FieldCanBeLocal") @SuppressLint("SdCardPath")
public class Logger {

    private static File robotPositionLog = new File("/sdcard/FIRST/RobotPosition.csv");
    //private static File robotPositionLog = new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LogTest.csv");
    private static FileWriter fileWriter;
    private static BufferedReader bufferedReader;

    public Logger() {}

    public void startLogging() {
        try {
            fileWriter = new FileWriter(robotPositionLog);
            fileWriter.write("Timestamp,SinceStart,X,Y,Theta,VelocityX,VelocityY,VelocityTheta,StoneInRobot,StoneClamped,ArmIsHome,ArmIsDown,ArmIsOut\n");
        } catch (Exception e) {e.printStackTrace();}
    }

    public void stopLogging() {try {fileWriter.close();} catch (Exception e) {e.printStackTrace();}}

    public void logData(double timeSinceSt, double x, double y, double theta, double velocityx, double velocityy, double velocitytheta, boolean stoneInRobot, boolean stoneClamped, boolean armIsHome, boolean armIsDown, boolean armIsOut) {
        SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss"); Date d = new Date();
        try {
            fileWriter.write(df.format(d)+","+timeSinceSt+","+x+","+y+","+theta+","+velocityx+","+velocityy+","+velocitytheta+","+stoneInRobot+","+stoneClamped+","+armIsHome+","+armIsDown+","+armIsOut+"\n");
        } catch (IOException ex) {ex.printStackTrace();}
    }

    public void flush() {
        try {fileWriter.flush();}
        catch (Exception e) {e.printStackTrace();}
    }

    public static double[] readPos() {
        String curLine; int lineNum = 0;
        double[] robotPos = new double[3];

        try {
            bufferedReader = new BufferedReader(new FileReader(robotPositionLog));
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
        } catch (IOException | NumberFormatException e) {
            robotPos[0] = 0; robotPos[1] = 0; robotPos[2] = 0;
            e.printStackTrace();
        }

        return robotPos;
    }
}
