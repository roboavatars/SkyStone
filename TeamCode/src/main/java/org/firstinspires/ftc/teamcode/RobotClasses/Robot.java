package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Splines.Waypoint;

@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    public Logger logger;

    // State booleans
    public boolean stoneInRobot = false;
    private boolean tryingToDeposit = false;
    private boolean downStacked = false;
    public boolean letGo = false;
    private boolean liftedUp = false;
    public boolean intakeManual = false;
    private boolean stoneInTimeSaved = false;
    private boolean armDownTimeSaved = false;

    public boolean yeetmode = false;
    public boolean cheesemode = false;
    private boolean firstLoop = true;

    // Class constants
    private final int armTicksUpdatePeriod = 7;
    private final int loggerUpdatePeriod = 2;
    public final double intakePower = 0.7;
    private final double armDownWaitTime = 200; //milliseconds
    private final double stonePushWaitTime = 1000;

    private int cycleCounter = 0;
    private double stoneInTime;
    private double armDownTime;

    // Velocity/acceleration stuff
    private double prevX, prevY, prevTh, xdot, ydot, w, prevxdot, prevydot, prevW, prevTime, xdotdot, ydotdot, a;
    public double startTime;

    //auto align stuff
    private final double AlignDistance = 5.7;
    public boolean isAutoAlign = false;
    public boolean isManualAlign = false;

    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta, boolean isRed) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta, isRed);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        capstoneDeposit = new CapstoneDeposit(op);
        logger = new Logger();

        this.op = op;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

    }

    public void update() {

        // increase cycle count
        cycleCounter++;

        // remember first loop time for time since start
        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        // update stacker
        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }

        if(cheesemode){
            if(stacker.getArmPosition()<30){
                stacker.setDepositControls(1.0, 60);
                stacker.unClampStone();
            }else{
                stacker.goHome();
                cheesemode = false;
            }

        }
        // teleop auto state changes
        else if (!yeetmode) {
            // return arm home after depositing
            if (!stoneInRobot && !tryingToDeposit) {
                stacker.goHome();
                stacker.unClampStone();
                if (!intakeManual) {
                    intake.setControls(intakePower);
                }
            }
            // return arm home after depositing when stone stuck in robot
            else if (stacker.isArmOut() && stoneInRobot) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                stacker.goHome();
                stacker.unClampStone();
            }
            // clamp stone after arm is move to clamping position
            else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit && !stacker.stoneClamped) {
                stacker.clampStone();
                if (!intakeManual) {
                    intake.setControls(0);
                }
            }
//            else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit && stacker.stoneClamped && stacker.getArmPosition()>-15 && !stacker.isArmMoving()) {
//                cheesemode = true;
//            }
            // when stone is intaked save time for clamping delay
            else if (stoneInRobot && !tryingToDeposit && stacker.isArmHome() && !stoneInTimeSaved && !armDownTimeSaved) {
                stoneInTime = System.currentTimeMillis();
                stoneInTimeSaved = true;
                intake.pushStoneIn();
            }
            // move arm to clamping position when delay is over
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit && stoneInTimeSaved && !armDownTimeSaved && (System.currentTimeMillis()-stoneInTime)>armDownWaitTime) {
                stacker.goDown();
                stoneInTimeSaved = false;

                armDownTime = System.currentTimeMillis();
                armDownTimeSaved = true;
            }
            //
            else if (stoneInRobot && stacker.isArmDown() && stacker.stoneClamped && !tryingToDeposit && !stoneInTimeSaved && armDownTimeSaved && (System.currentTimeMillis()-armDownTime)>stonePushWaitTime) {
                intake.stoneServoHome();
                armDownTimeSaved = false;
            }
            // check if we should downstack
            else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && !stacker.isDownStacked() && !downStacked && letGo) {
                stacker.downStack();
                downStacked = true;
            }
            // check if we should unclamp and move lift up
            else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && stacker.isDownStacked() && letGo) {
                stacker.unClampStone();
                stacker.liftUp();
                liftedUp = true;
            }
            // set depositing variables to false after lift is up
            else if (tryingToDeposit && stacker.isArmOut() && stacker.isLiftUp() && downStacked && letGo && liftedUp) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                liftedUp = false;
                stacker.nextLevel();
            }
            // check if we should go to deposit position
            else if (tryingToDeposit && !downStacked) {
                stacker.deposit();
                /*if (stacker.currentStackHeight > 0) {
                    grabber.extendRangeSensor();
                }*/
            }

//            if (tryingToDeposit && (!letGo /*|| depositAuto*/) && cycleCounter % 2 == 0 && stacker.currentStackHeight > 0 && !isManualAlign) {
//                grabber.extendRangeSensor();
//                double distance = grabber.getDistance();
//                op.telemetry.addData("align dist", distance);
//                if (Math.abs(distance - AlignDistance) < 7) {
//                    drivetrain.setControls(-0.25 * (distance - AlignDistance), -0.15, 0);
//                } else {
//                    drivetrain.setControls(0, 0, 0);
//                }
//                isAutoAlign = true;
//
////                if (depositAuto) {
////                    if (Math.abs(distance - AlignDistance) < 0.5 || Math.abs(distance - AlignDistance) > 12) {
////                        letGo = true;
////                    }
////                }
//            } else if (cycleCounter % 2 == 0) {
//                isAutoAlign = false;
//            }
        }
        // auto auto state changes
        else {
            // return arm home after depositing
            if(!tryingToDeposit && !stoneInRobot){
                stacker.goHome();
                intake.setControls(intakePower);
            }
            // when stone is intaked save time for clamping delay
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit && !stoneInTimeSaved && !armDownTimeSaved) {
                stoneInTime = System.currentTimeMillis();
                stoneInTimeSaved = true;
                intake.pushStoneIn();
            }
            // move arm to clamping position when delay is over
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit && stoneInTimeSaved && !armDownTimeSaved && (System.currentTimeMillis()-stoneInTime)>armDownWaitTime) {
                stacker.goDown();
                stoneInTimeSaved = false;

                armDownTime = System.currentTimeMillis();
                armDownTimeSaved = true;
            }
            else if (stoneInRobot && stacker.isArmDown() && stacker.stoneClamped && !tryingToDeposit && !stoneInTimeSaved && armDownTimeSaved && (System.currentTimeMillis()-armDownTime)>stonePushWaitTime) {
                intake.stoneServoHome();
                armDownTimeSaved = false;
            }
            // clamp stone after arm is moved to clamping position
            else if (stoneInRobot && stacker.isArmDown() && !stacker.stoneClamped && !tryingToDeposit) {
                stacker.clampStone();
                intake.setControls(0);
            }
            // check if we should deposit stone
            else if (stoneInRobot && tryingToDeposit && !stacker.atAutoDepositPos()) {
                intake.stoneServoHome();
                stacker.depositAuto();
            }
            // unclamp stone after arm is past certain thershold
            //TODO fix this hardcoded position
            else if (!stoneInRobot && tryingToDeposit && stacker.getArmPosition()>850) {
                stacker.unClampStone();
                tryingToDeposit = false;
                intake.setControls(intakePower);
            }
        }

        // update drivetrain
        drivetrain.updatePose();
        stoneInRobot = drivetrain.stoneInRobot;

        // calculating velocity/acceleration
        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        xdot = (drivetrain.x - prevX) / timeDiff;
        ydot = (drivetrain.y - prevY) / timeDiff;
        w = (drivetrain.currentheading - prevTh) / timeDiff;
        xdotdot = (xdot - prevxdot) / timeDiff;
        ydotdot = (ydot - prevydot) / timeDiff;
        a = (w - prevW) / timeDiff;

        // log data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,xdot,ydot,w,xdotdot,ydotdot,a,stoneInRobot,stacker.stoneClamped,tryingToDeposit,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
        }

        // remember old values so calc velocity/acceleration
        prevX = drivetrain.x;
        prevY = drivetrain.y;
        prevTh = drivetrain.currentheading;
        prevTime = curTime;
        prevydot = ydot;
        prevxdot = xdot;
        prevW = w;

        // telemetry
        drawRobot(drivetrain.x, drivetrain.y, drivetrain.currentheading);
        addPacket("X", drivetrain.x);
        addPacket("Y", drivetrain.y);
        addPacket("Theta", drivetrain.currentheading);
        addPacket("is stone in robot", stoneInRobot);
        addPacket("distance", drivetrain.distance);
        addPacket("robot velocity", Math.sqrt(Math.pow(xdot,2) + Math.pow(ydot, 2)));
        addPacket("arm", "home:" + stacker.isArmHome() + " down:" + stacker.isArmDown() + " out:" + stacker.isArmOut() + " deposit:" + tryingToDeposit);
        addPacket("update frequency(hz)", 1/timeDiff);
        addPacket("arm ticks" , stacker.getArmPosition());
        addPacket("deltapod1: ", drivetrain.deltapod1);
        addPacket("deltapod2: ", drivetrain.deltapod2);
        addPacket("deltapod3: ", drivetrain.deltapod3);

        sendPacket();
    }

    public void deposit() {
        if (!stacker.isArmOut()) {
            tryingToDeposit = true;
        }
    }

    public void depositAuto() {
        if (!stacker.isArmOut() && stoneInRobot) {
            tryingToDeposit = true;
        }
    }

    public Waypoint currentRobotWaypoint(){
        return new Waypoint(drivetrain.x, drivetrain.y, drivetrain.currentheading,
                xdot, ydot, xdotdot, ydotdot, 0);
    }

    public void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public void drawRobot(double robotx, double roboty, double robottheta) {
        double r = 9 * Math.sqrt(2);
        double pi = Math.PI;
        double x = 72 - roboty;
        double y = robotx - 72;
        double theta = pi/2 + robottheta;
        double[] ycoords = {r * Math.sin(pi / 4 + theta) + y, r * Math.sin(3 * pi / 4 + theta) + y, r * Math.sin(5 * pi / 4 + theta) + y, r * Math.sin(7 * pi / 4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi / 4 + theta) + x, r * Math.cos(3 * pi / 4 + theta) + x, r * Math.cos(5 * pi / 4 + theta) + x, r * Math.cos(7 * pi / 4 + theta) + x};
        if (stoneInRobot) {packet.fieldOverlay().setFill("yellow").fillPolygon(xcoords,ycoords);}
        else {packet.fieldOverlay().setFill("green").fillPolygon(xcoords,ycoords);}
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public void log(String message) {
        Log.w("robot", " ");
        Log.w("robot", message + " -------------------------------------------------------------------------");
    }

    public double getBatteryVoltage() {
        double result = -1;
        for (VoltageSensor sensor : op.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
