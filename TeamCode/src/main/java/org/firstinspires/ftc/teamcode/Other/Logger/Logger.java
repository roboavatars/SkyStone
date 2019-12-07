package org.firstinspires.ftc.teamcode.Other.Logger;

import android.annotation.SuppressLint;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@SuppressLint("SimpleDateFormat")
public class Logger {

    private File logFile;
    private final String filePath = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Logger/Log.csv";

    public Logger() {
        try {
            logFile = new File(filePath);
            if (!(logFile.exists())) {
                System.out.println("new file");
                SimpleDateFormat df = new SimpleDateFormat("dd/MM/yyyy");
                Date d  = new Date();

                BufferedWriter logSetup = new BufferedWriter(new FileWriter(logFile));
                logSetup.write("Time (Created On " + df.format(d) + "),Level,Entry\n");
                logSetup.close();
            }
        } catch (IOException ex) {
            System.out.println("ioe");
            ex.printStackTrace();
        }
    }

    public void addEntry(String logLevel, String entry) {
        SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss");
        Date d = new Date();
        try {
            BufferedWriter logWriter = new BufferedWriter(new FileWriter(logFile, true));
            logWriter.write(df.format(d) + "," + logLevel + "," + entry + "\n");
            logWriter.close();
        } catch (IOException ex) {
            System.out.println("ioe");
            ex.printStackTrace();
        }
    }
}
