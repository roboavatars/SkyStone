package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public Rev2mDistanceSensor stoneSensor;

    // State booleans
    public boolean stoneInRobot = false;
    private boolean armIsOut = false;

    //TODO create clampedOnFoundation methods
    private boolean clampedOnFoundation = false;
    private boolean downstacked = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 20;
    private final int stoneValidationDistance = 6;
    private final int armTicksUpdatePeriod = 10;

    private int cycleCounter = 0;
    private int z = 20;

    private static File robotPositionLog;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);

        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");
    }

    public void update() {
        // increase cycle count
        cycleCounter++;

        // update stoneInRobot boolean
        if (cycleCounter % stoneSensorUpdatePeriod == 0) {
            stoneInRobot = stoneSensor.getDistance(DistanceUnit.INCH) < stoneValidationDistance;
        }

        // update arm
        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }

        // check states----------------------
        // check if ready to collect stones
        if (!stoneInRobot && isHome()) {
            intake.setControls(1);
        }
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && isHome()) {
            if (z > 0) {
                z--;

            } else {
                intake.setControls(0);
                stacker.goDown();
                z = 20;
            }

        }
        // check if stone should be clamped
        else if (stoneInRobot && stacker.isArmDown() && !stacker.stoneClamped) {
            stacker.clampStone();
        }
        // check if stone should be unclamped
        else if (!stacker.isArmMoving() && downstacked) {
            stacker.unClampStone();
            stacker.setLiftControls(0.8, Math.min(stacker.getLiftPosition() + 100, 1285));
            downstacked = false;
        }

        drivetrain.updatePose();
    }

    public boolean isHome(){
        return (stacker.isArmHome() && stacker.isLiftHome() && !stacker.stoneClamped);
    }

    public void swapArmState() {
        if (!stacker.isArmOut()) {
            stacker.deposit();
            stacker.nextLevel();
        } else if (!stacker.stoneClamped) {
            stacker.goHome();
        }
    }

    public void deposit() {
        if (stacker.isArmOut() && stacker.stoneClamped) {
            stacker.downStack();
            downstacked = true;
        }
    }

    //file stuff
    /*public void writePos(double x, double y, double theta) {
        robotPositionLog = new File("/sdcard/FIRST/RobotPosition.txt");
        try {
            FileWriter fileWriter = new FileWriter(robotPositionLog);
            fileWriter.write(x + "\n");
            fileWriter.write(y + "\n");
            fileWriter.write(theta + "");
            fileWriter.close();
        } catch (Exception e) {e.printStackTrace();}
    }

    public static double[] readPos() {
        String curLine;
        double[] robotPos = new double[3];

        int counter = 0;
        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(robotPositionLog));

            while ((curLine = bufferedReader.readLine()) != null) {
                robotPos[counter] = Double.parseDouble(curLine);
                counter++;
            }
            bufferedReader.close();
        } catch (FileNotFoundException e) {
            robotPos[0] = 0; robotPos[1] = 0; robotPos[2] = 0;
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return robotPos;
    }

    public static void deletePosFile() {
        robotPositionLog.delete();
    }*/
}
