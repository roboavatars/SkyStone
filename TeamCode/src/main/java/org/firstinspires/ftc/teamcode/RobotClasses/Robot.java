package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class Robot {
    
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public Rev2mDistanceSensor stoneSensor;
    
    boolean rangeSensorEnabled;
    boolean stoneInRobot;

    private File robotPositionLog = new File(new File("/sdcard/FIRST/"), "RobotPositionLog");
    
    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        
        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");
    }
    
    public void update() {drivetrain.updatePose();}
    public void writePos(double x, double y, double theta) {
        try {
            FileWriter fileWriter = new FileWriter(robotPositionLog);
            fileWriter.write(x + "\n");
            fileWriter.write(y + "\n");
            fileWriter.write(theta + "\n");
            fileWriter.write("\n");
            fileWriter.close();
        } catch (Exception e) {e.printStackTrace();}
    }
    public String[] readPos() {
        String line;
        String[] lines = new String[3];
        int counter = 0;
        try {
            FileReader fileReader = new FileReader(robotPositionLog);
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            while ((line = bufferedReader.readLine()) != null) {
                lines[counter] = line;
                counter = (counter + 1) % 3;

            }
            fileReader.close();
        } catch (Exception e) {e.printStackTrace();}
        return lines;
    }
}
