package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;

import java.util.Date;

public class Robot {
    
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public Rev2mDistanceSensor stoneSensor;
    
    boolean rangeSensorEnabled;
    boolean stoneInRobot;

    private File file = new File(new File("/sdcard/FIRST/"), "RobotDataLog");
    
    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        
        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");
    }
    
    public void update() {drivetrain.updatePose();}
    public void logData(String category, String text) {
        Date time = new Date();
        try {
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(time + "-" + category + "----" + text);
            fileWriter.close();
        } catch (Exception e) {e.printStackTrace();}
    }
}
