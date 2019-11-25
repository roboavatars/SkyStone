package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    public Rev2mDistanceSensor stoneSensor;

    public PositionLogger logger;

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
        logger = new PositionLogger();

        this.op = op;

        stoneSensor = op.hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");

        if (stoneSensor.getDistance(DistanceUnit.INCH) > 18) {
            op.telemetry.addData("DISTANCE SENSOR VALUE TOO HIGH", stoneSensor.getDistance(DistanceUnit.INCH));
            op.telemetry.update();
        }
    }

    public void update() {

        op.telemetry.addData("stone in robot", stoneInRobot);
        op.telemetry.addData("arm is out", stacker.isArmOut());
        op.telemetry.addData("arm is home", stacker.isArmHome());
        op.telemetry.addData("arm is down", stacker.isArmDown());
        op.telemetry.addData("stone clamped", stacker.stoneClamped);
        op.telemetry.update();

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
            intake.setControls(1);
        }
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && isHome()) {
            if (z > 0) {
                z--;

            } else {
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
        else if (!stacker.isArmMoving() && downStacked && x>15) {
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
            logger.writePos(drivetrain.x, drivetrain.y, drivetrain.currentheading);
        }
        if (cycleCounter % flushUpdatePeriod == 0) {
            logger.flush();
        }
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
        depositAuto = !depositAuto;
    }
}
