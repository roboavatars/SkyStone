package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    private Rev2mDistanceSensor stoneSensor;
    public Logger logger;

    private double prevX, prevY, prevTh, velocityX, velocityY, velocityTh, prevTime;
    private double startTime;

    // State booleans
    public boolean stoneInRobot = false;
    private boolean depositAuto = false;
    public boolean isOutTake = false;

    //TODO create clampedOnFoundation methods
    private boolean clampedOnFoundation = false;
    private boolean downStacked = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 7;
    private final int stoneValidationDistance = 6;
    private final int armTicksUpdatePeriod = 10;
    private final int loggerUpdatePeriod = 2;
    private final int flushUpdatePeriod = 5000;

    private int cycleCounter = 0;
    private int z = 10;
    private int x = 0;

    private LinearOpMode op;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        capstoneDeposit = new CapstoneDeposit(op);
        logger = new Logger();
        startTime = System.currentTimeMillis();

        this.op = op;

        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");

        if (stoneSensor.getDistance(DistanceUnit.INCH) > 18) {
            op.telemetry.addData("DISTANCE SENSOR VALUE TOO HIGH", stoneSensor.getDistance(DistanceUnit.INCH));
            op.telemetry.update();
        }
    }

    public void update() {

        isOutTake = false;

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

        //update downstack time variable
        if(downStacked){
            x++;
        }

        //expel block if arm is out
        if (stoneInRobot && stacker.isArmOut()) {
            expelStone();
        }

        // check states----------------------
        // check if ready to collect stones
        if (!stoneInRobot && isHome() && !isOutTake) {
            Log.w("graph", "intake on called");
            intake.setControls(1);
        }
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && isHome()) {
            if (z > 0) {
                z--;

            } else {
                Log.w("graph", "intake off called");
                intake.setControls(0);
                stacker.goDown();
                z = 10;
            }

        }
        // check if stone should be clamped
        else if (stoneInRobot && stacker.isArmDown() && !stacker.stoneClamped) {
            stacker.clampStone();
        }
        // check if stone should be unclamped
        else if (!stacker.isArmMoving() && downStacked && x > 15) {
            stacker.unClampStone();
            stacker.setLiftControls(0.8, Math.min(stacker.getLiftPosition() + 1200, 2400));

            x = 0;
            downStacked = false;
        }
        // deposit stone in auto
        else if (depositAuto) {
            if(!stacker.isArmOut()){
                stacker.deposit();
            }
            else if (!stacker.isArmMoving()) {
                stacker.unClampStone();
                stacker.goHome();
                depositAuto = false;
            }
        }

        drivetrain.updatePose();
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,velocityX,velocityY,velocityTh,stoneInRobot,stacker.stoneClamped,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
        }
        if (cycleCounter % flushUpdatePeriod == 0) {
            logger.flush();
        }

        double curTime = (double) System.currentTimeMillis();
        double timeDiff = curTime - prevTime;
        velocityX = (drivetrain.x - prevX) / timeDiff;
        velocityY = (drivetrain.y - prevY) / timeDiff;
        velocityTh = (drivetrain.currentheading - prevTh) / timeDiff;
        prevX = drivetrain.x; prevY = drivetrain.y; prevTh = drivetrain.currentheading; prevTime = curTime;

        op.telemetry.addData("Robot x", drivetrain.x);
        op.telemetry.addData("Robot y", drivetrain.y);
        op.telemetry.addData("Robot theta", drivetrain.currentheading);
        op.telemetry.addData("Robot velocity", velocityX+", "+velocityY+", "+ velocityTh +" ("+timeDiff+")");
        op.telemetry.addData("States", "");
        op.telemetry.addData("stone in robot", stoneInRobot);
        op.telemetry.addData("stone clamped", stacker.stoneClamped);
        op.telemetry.addData("arm is out", stacker.isArmOut());
        op.telemetry.addData("arm is home", stacker.isArmHome());
        op.telemetry.addData("arm is down", stacker.isArmDown());
    }

    public boolean isHome(){
        return (stacker.isArmHome() && stacker.isLiftHome() && grabber.isGrabberHome() && !stacker.stoneClamped);
    }

    public void swapArmStateTeleop() {
        if (!stacker.isArmOut()) {
            stacker.deposit();
            stacker.nextLevel();
        } else if (!stacker.stoneClamped) {
            stacker.goHome();
        }
    }

    public void deposit() {
        if (stacker.isArmOut() && stacker.stoneClamped && !downStacked) {
            stacker.downStack();
            downStacked = true;
        }
    }

    public void expelStone() {
        stacker.goHome();
        stacker.unClampStone();
        intake.setControls(-1);
        isOutTake = true;
    }

    public void depositAuto() {
        Log.w("graph", "deposit stone called");
        depositAuto = !depositAuto;
    }
}
