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
    public Rev2mDistanceSensor stoneSensor;

    public PositionLogger logger;

    // State booleans
    public boolean stoneInRobot = false;

    //TODO create clampedOnFoundation methods
    private boolean clampedOnFoundation = false;
    private boolean downStacked = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 20;
    private final int stoneValidationDistance = 6;
    private final int armTicksUpdatePeriod = 10;

    private int cycleCounter = 0;
    private int z = 10;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        logger = new PositionLogger();

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
                z = 10;
            }

        }
        // check if stone should be clamped
        else if (stoneInRobot && stacker.isArmDown() && !stacker.stoneClamped) {
            stacker.clampStone();
        }
        // check if stone should be unclamped
        else if (!stacker.isArmMoving() && downStacked) {
            stacker.unClampStone();
            stacker.setLiftControls(0.8, Math.min(stacker.getLiftPosition() + 150, 1285));
            stacker.setDepositControls(0.2, stacker.getArmPosition() - 100);
            downStacked = false;
        }

        drivetrain.updatePose();
        logger.writePos(drivetrain.x, drivetrain.y, drivetrain.currentheading);
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
            downStacked = true;
        }
    }

    public void expelStone() {
        stacker.goHome();
        stacker.unClampStone();
        intake.setControls(-1);
    }
}
