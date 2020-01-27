package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    public Logger logger;

    private double prevX, prevY, prevTh, velocityX, velocityY, velocityTh, prevTime;
    private double startTime;

    // State booleans
    public boolean stoneInRobot = false;
    private boolean tryingToDeposit = false;
    private boolean downStacked = false;
    public boolean letGo = false;
    private boolean liftedUp = false;
    private boolean depositAuto = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 7;
    private final int stoneValidationDistance = 3;
    private final int armTicksUpdatePeriod = 5;
    private final int loggerUpdatePeriod = 2;
    private final int flushUpdatePeriod = 5000;
    private final double AlignDistance = 5.7;

    private int cycleCounter = 0;
    public boolean isAutoAlign = false;
    public boolean intakeManual = false;
    public boolean isManualAlign = false;

    private LinearOpMode op;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta, boolean isRedAuto) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta, isRedAuto);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        capstoneDeposit = new CapstoneDeposit(op);
        logger = new Logger();
        startTime = System.currentTimeMillis();

        this.op = op;

//        if (stoneSensor.getDistance(DistanceUnit.INCH) > 18) {
//            op.telemetry.addData("DISTANCE SENSOR VALUE TOO HIGH", stoneSensor.getDistance(DistanceUnit.INCH));
//            op.telemetry.update();
//        }
    }

    public void update() {

        // increase cycle count
        cycleCounter++;

        // update stoneInRobot boolean
//        if (cycleCounter % stoneSensorUpdatePeriod == 0) {
//            stoneInRobot = stoneSensor.getDistance(DistanceUnit.INCH) < stoneValidationDistance;
//        }

        // update arm
        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }
        if(depositAuto && stacker.currentStackHeight == 0){
            letGo = true;
        }

        // check states----------------------
        // check if ready to collect stones
        if (!stoneInRobot && !tryingToDeposit) {
            stacker.goHome();
            stacker.unClampStone();
            if(!intakeManual){
                intake.setControls(0.5);
            }
        }
        else if (stacker.isArmOut() && stoneInRobot && !depositAuto){
            tryingToDeposit = false;
            downStacked = false;
            letGo = false;
            depositAuto = false;
            stacker.goHome();
            stacker.unClampStone();
        }
        else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit) {
            stacker.clampStone();
            if(!intakeManual){
                intake.setControls(0);
            }
        }
        else if(stoneInRobot && !tryingToDeposit){
            if(!intakeManual){
                intake.setControls(0);
            }

        }
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && !tryingToDeposit && !stacker.isArmOut()) {
            stacker.goDown();
        }
        //check if we should downstack
        else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && !stacker.isDownStacked() && !downStacked && letGo) {
            stacker.downStack();
            downStacked = true;
            op.telemetry.addLine("trying to downstack");
        }
        else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && stacker.isDownStacked() && letGo) {
            stacker.unClampStone();
            stacker.liftUp();
            liftedUp = true;
            op.telemetry.addLine("unclamped stone");
        }
        else if (tryingToDeposit && stacker.isArmOut() && stacker.isLiftUp() && downStacked && letGo && liftedUp) {
            tryingToDeposit = false;
            downStacked = false;
            letGo = false;
            depositAuto = false;
            liftedUp = false;
            stacker.nextLevel();
        }
        else if (tryingToDeposit && !downStacked) {
            stacker.deposit();
            if(stacker.currentStackHeight > 0){
                grabber.extendRangeSensor();
            }

        }

        if (tryingToDeposit && (!letGo || depositAuto) && cycleCounter % 2 == 0 && stacker.currentStackHeight > 0 && !isManualAlign) {
            grabber.extendRangeSensor();
            double distance = grabber.getDistance();
            op.telemetry.addData("align dist", distance);
            if (Math.abs(distance-AlignDistance) < 7) {
                drivetrain.setControls(-0.25*(distance-AlignDistance),-0.15,0);
            }
            else {
                drivetrain.setControls(0,0,0);
            }
            isAutoAlign = true;

            if(depositAuto){
                if(Math.abs(distance-AlignDistance)<0.5 || Math.abs(distance-AlignDistance)>12){
                    letGo = true;
                }
            }
        }
        else if (cycleCounter % 2 == 0) {
            isAutoAlign = false;
        }


        drivetrain.updatePose();
        stoneInRobot = drivetrain.stoneInRobot;

        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,velocityX,velocityY,velocityTh,stoneInRobot,stacker.stoneClamped,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
        }
        if (cycleCounter % flushUpdatePeriod == 0) logger.flush();

        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        velocityX = (drivetrain.x - prevX) / timeDiff;
        velocityY = (drivetrain.y - prevY) / timeDiff;
        velocityTh = (drivetrain.currentheading - prevTh) / timeDiff;
        prevX = drivetrain.x; prevY = drivetrain.y; prevTh = drivetrain.currentheading; prevTime = curTime;

        op.telemetry.addData("Robot x", drivetrain.x);
        op.telemetry.addData("Robot y", drivetrain.y);
        op.telemetry.addData("Robot theta", drivetrain.currentheading);
        op.telemetry.addData("is stone in robot", stoneInRobot);
        op.telemetry.addData("is lift up", stacker.isLiftUp());
    }

    public void deposit() {
        if (!stacker.isArmOut()) {
            tryingToDeposit = true;
        }
    }

    public void depositAuto() {
        if (!stacker.isArmOut()) {
            tryingToDeposit = true;
            depositAuto = true;
        }
    }
    public void log(String message) {
        Log.w("robot", " ");
        Log.w("robot", message + " -------------------------------------------------------------------------");
    }
}
