package org.firstinspires.ftc.teamcode.RobotClasses;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@SuppressWarnings("FieldCanBeLocal")
public class PositionLogger {

    public static File robotPositionLog = new File("/sdcard/FIRST/RobotPosition.txt");
    public static FileWriter fileWriter;
    public static BufferedReader bufferedReader;

    public PositionLogger() {
    }

    public void startLogging() {try {fileWriter = new FileWriter(robotPositionLog);} catch (Exception e) {e.printStackTrace();}}

    public void stopLogging() {try {fileWriter.close();} catch (Exception e) {e.printStackTrace();}}

    public void writePos(double x, double y, double theta) {
        try {
            fileWriter.write("\n" + x);
            fileWriter.write("\n" + y);
            fileWriter.write("\n" + theta);
            fileWriter.flush();
        } catch (Exception e) {e.printStackTrace();}
    }

    public static double[] readPos() {
        String curLine;
        double[] robotPos = new double[3];
        double[] robotPosFinal = new double[3];

        boolean firstLine = true;
        int counter = 2;
        try {
            bufferedReader = new BufferedReader(new FileReader(robotPositionLog));
            while ((curLine = bufferedReader.readLine()) != null) {
                if (firstLine) {firstLine = false;}
                else {robotPos[counter] = Double.parseDouble(curLine);}
                if (counter == 2 && !firstLine) {
                    robotPosFinal[0] = robotPos[0];
                    robotPosFinal[1] = robotPos[1];
                    robotPosFinal[2] = robotPos[2];
                }
                counter = (counter + 1) % 3;
            }
            bufferedReader.close();
        } catch (FileNotFoundException e) {
            robotPosFinal[0] = 0; robotPosFinal[1] = 0; robotPosFinal[2] = 0;
            e.printStackTrace();
        } catch (IOException e) {e.printStackTrace();}

        return robotPosFinal;
    }
}
