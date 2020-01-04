package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    private boolean tryingToDeposit = false;

    //TODO create clampedOnFoundation methods
    private boolean downStacked = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 7;
    private final int stoneValidationDistance = 3;
    private final int armTicksUpdatePeriod = 5;
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
        if (!stoneInRobot && !tryingToDeposit) {
            stacker.goHome();
            stacker.unClampStone();
        }
        else if(stoneInRobot && stacker.isArmDown() ){
            stacker.clampStone();
        }
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && !tryingToDeposit && !stacker.isArmOut()) {
            stacker.goDown();
        }
        //check if we should downstack
        else if(tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && !stacker.isDownStacked()){
            stacker.downStack();
            op.telemetry.addLine("trying to downstack");
        }
        else if(tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving()){
            stacker.unClampStone();
            stacker.liftUp();
            tryingToDeposit = false;
            op.telemetry.addLine("unclamped that shit");
        }


        drivetrain.updatePose();
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,velocityX,velocityY,velocityTh,stoneInRobot,stacker.stoneClamped,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
        }
        if (cycleCounter % flushUpdatePeriod == 0) {
            logger.flush();
        }

        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        velocityX = (drivetrain.x - prevX) / timeDiff;
        velocityY = (drivetrain.y - prevY) / timeDiff;
        velocityTh = (drivetrain.currentheading - prevTh) / timeDiff;
        prevX = drivetrain.x;
        prevY = drivetrain.y;
        prevTh = drivetrain.currentheading;
        prevTime = curTime;

        op.telemetry.addData("Robot x", drivetrain.x);
        op.telemetry.addData("Robot y", drivetrain.y);
        op.telemetry.addData("Robot theta", drivetrain.currentheading);
        op.telemetry.addData("stone in robot", stoneInRobot);
        op.telemetry.addData("arm down", stacker.isArmDown());

    }


    public void deposit() {
        if (!stacker.isArmOut()) {
            stacker.deposit();
            stacker.nextLevel();
            tryingToDeposit = true;

        }
    }

}
