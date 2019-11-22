package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class Robot {

    //subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public Rev2mDistanceSensor stoneSensor;

    //state booleans
    private boolean stoneInRobot = false;
    private boolean armIsOut = false;

    //TODO create clampedonFoundation methods
    private boolean clampedOnFoundation = false;
    private boolean downstacked = false;


    //class constants
    private final int stoneSensorUpdatePeriod = 20;
    private final int stoneValidationDistance = 6;
    private final int armTicksUpdatePeriod = 10;


    private int cycleCounter = 0;

    private static File robotPositionLog = new File(new File("/sdcard/FIRST/"), "RobotPositionLog");
    
    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        
        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");
    }
    
    public void update() {
        //increase cycle count
        cycleCounter++;

        //update stoneInRobot boolean
        if(cycleCounter%stoneSensorUpdatePeriod == 0){
            stoneInRobot = stoneSensor.getDistance(DistanceUnit.INCH) < stoneValidationDistance;
        }

        //update arm is out boolean
        if((cycleCounter+3)%armTicksUpdatePeriod == 0){
            stacker.update();
        }

        //check states

        //check if ready to collect
        if(!stoneInRobot && ishome()){
            intake.setControls(1);
        }
        //start clamping procedure
        else if(stoneInRobot && ishome()){
            intake.setControls(0);
            stacker.goDown();
        }
        else if(stoneInRobot && stacker.isArmHome() && !stacker.stoneClamped){
            stacker.clampStone();
        }
        else if(!stacker.isArmMoving() && downstacked){
            stacker.unClampStone();
            downstacked = false;
        }

        drivetrain.updatePose();
    }
    public boolean ishome(){
        //TODO add lift home position stuff
        return (stacker.isArmHome() && !stacker.stoneClamped);
    }
    public void swapArmState(){
        if(stacker.isArmOut()){
            stacker.deposit();
        }
        else if(!stacker.stoneClamped){
            stacker.goHome();
        }
    }
    public void deposit(){
        if(stacker.isArmOut() && stacker.stoneClamped){
            stacker.downStack();
            downstacked = true;
        }
    }




    /*//file stuff
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
    public static Double[] readPos() {
        String line;
        Double[] lines = new Double[3];
        int counter = 0;
        try {
            FileReader fileReader = new FileReader(robotPositionLog);
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            while ((line = bufferedReader.readLine()) != null) {
                lines[counter] = Double.parseDouble(line);
                counter = (counter + 1) % 3;

            }
            fileReader.close();
        } catch (Exception e) {e.printStackTrace();}
        return lines;
    }*/
}
