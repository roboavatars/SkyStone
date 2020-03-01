package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Splines.Waypoint;

@SuppressWarnings("FieldCanBeLocal")
//@Config
public class Robot {

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    public Logger logger;

    // State Booleans
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
    private final double armDownWaitTime = 200; //Milliseconds
    private final double stonePushWaitTime = 1000;

    private int cycleCounter = 0;
    private double stoneInTime;
    private double armDownTime;

    // Velocity/Acceleration Stuff
    private double prevX, prevY, prevTh, xdot, ydot, w, prevxdot, prevydot, prevW, prevTime, xdotdot, ydotdot, a;
    public double startTime;

    // OpMode Stuff
    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    // Constructor
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
        // Increase Cycle Count
        cycleCounter++;

        // Remember First Loop Time for Time Since Start
        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        // Update Stacker
        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }

        // In Case Arm Does Not Clamp
        if (cheesemode) {
            if(stacker.getArmPosition() < 30){
                stacker.setDepositControls(1.0, 60);
                stacker.unClampStone();
            } else {
                stacker.goHome();
                cheesemode = false;
            }
        }

        // Teleop Auto State Changes
        else if (!yeetmode) {
            // Return Arm Home After Depositing
            if (!stoneInRobot && !tryingToDeposit) {
                stacker.goHome();
                stacker.unClampStone();
                if (!intakeManual) {
                    intake.setControls(intakePower);
                }
            }

            // Return Arm Home After Depositing when Stone Stuck in Robot
            else if (stacker.isArmOut() && stoneInRobot) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                stacker.goHome();
                stacker.unClampStone();
            }

            // Clamp Stone Once Intaked
            else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit && !stacker.stoneClamped) {
                stacker.clampStone();
                if (!intakeManual) {
                    intake.setControls(0);
                }
            }

//            else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit && stacker.stoneClamped && stacker.getArmPosition()>-15 && !stacker.isArmMoving()) {
//                cheesemode = true;
//            }

            // When Stone is Intaked Save Time for Clamping Delay
            else if (stoneInRobot && !tryingToDeposit && stacker.isArmHome() && !stoneInTimeSaved && !armDownTimeSaved) {
                stoneInTime = System.currentTimeMillis();
                stoneInTimeSaved = true;
                intake.pushStoneIn();
            }

            // Move Arm to Clamping Position When Delay is Over
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit && stoneInTimeSaved && !armDownTimeSaved && (System.currentTimeMillis()-stoneInTime)>armDownWaitTime) {
                stacker.goDown();
                stoneInTimeSaved = false;

                armDownTime = System.currentTimeMillis();
                armDownTimeSaved = true;
            }

            // Move Intake Servo Once Stone is Clamped
            else if (stoneInRobot && stacker.isArmDown() && stacker.stoneClamped && !tryingToDeposit && !stoneInTimeSaved && armDownTimeSaved && (System.currentTimeMillis()-armDownTime)>stonePushWaitTime) {
                intake.stoneServoHome();
                armDownTimeSaved = false;
            }

            // Check if We Should Downstack
            else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && !stacker.isDownStacked() && !downStacked && letGo) {
                stacker.downStack();
                downStacked = true;
            }

            // Check if We Should Unclamp and Move Lift Up
            else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && stacker.isDownStacked() && letGo) {
                stacker.unClampStone();
                stacker.liftUp();
                liftedUp = true;
            }

            // Set Depositing Variables to False After Lift is Up
            else if (tryingToDeposit && stacker.isArmOut() && stacker.isLiftUp() && downStacked && letGo && liftedUp) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                liftedUp = false;
                stacker.nextLevel();
            }

            // Check if We Should Go to Deposit Position
            else if (tryingToDeposit && !downStacked) {
                stacker.deposit();
            }
        }
        // Auto State Changes
        else {
            // return arm home after depositing
            if(!tryingToDeposit && !stoneInRobot){
                stacker.goHome();
                intake.setControls(intakePower);
            }

            // When Stone is Intaked Save Time for Clamping Delay
            else if (stoneInRobot && !tryingToDeposit && stacker.isArmHome() && !stoneInTimeSaved && !armDownTimeSaved) {
                stoneInTime = System.currentTimeMillis();
                stoneInTimeSaved = true;
                intake.pushStoneIn();
            }

            // Move Arm to Clamping Position When Delay is Over
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit && stoneInTimeSaved && !armDownTimeSaved && (System.currentTimeMillis()-stoneInTime)>armDownWaitTime) {
                stacker.goDown();
                stoneInTimeSaved = false;

                armDownTime = System.currentTimeMillis();
                armDownTimeSaved = true;
            }

            // Move Intake Servo Once Stone is Clamped
            else if (stoneInRobot && stacker.isArmDown() && stacker.stoneClamped && !tryingToDeposit && !stoneInTimeSaved && armDownTimeSaved && (System.currentTimeMillis()-armDownTime)>stonePushWaitTime) {
                intake.stoneServoHome();
                armDownTimeSaved = false;
            }

            // Clamp Stone After Arm is Moved to Clamping Position
            else if (stoneInRobot && stacker.isArmDown() && !stacker.stoneClamped && !tryingToDeposit) {
                stacker.clampStone();
                intake.setControls(0);
            }

            // Check if We Should Deposit Stone
            else if (stoneInRobot && tryingToDeposit && !stacker.atAutoDepositPos()) {
                intake.stoneServoHome();
                stacker.depositAuto();
            }

            // Unclamp Stone After Arm is Past Certain Threshold
            //TODO: fix this hardcoded position
            else if (!stoneInRobot && tryingToDeposit && stacker.getArmPosition() > 850) {
                stacker.unClampStone();
                tryingToDeposit = false;
                intake.setControls(intakePower);
            }
        }

        // Update Drivetrain
        drivetrain.updatePose();
        stoneInRobot = drivetrain.stoneInRobot;

        // Calculating Velocity/Acceleration
        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        xdot = (drivetrain.x - prevX) / timeDiff;
        ydot = (drivetrain.y - prevY) / timeDiff;
        w = (drivetrain.currentheading - prevTh) / timeDiff;
        xdotdot = (xdot - prevxdot) / timeDiff;
        ydotdot = (ydot - prevydot) / timeDiff;
        a = (w - prevW) / timeDiff;

        // Log Data
        if (cycleCounter % loggerUpdatePeriod == 0) {
            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,xdot,ydot,w,xdotdot,ydotdot,a,stoneInRobot,stacker.stoneClamped,tryingToDeposit,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
        }

        // Remember Old Values to Calculate Velocity/Acceleration
        prevX = drivetrain.x;
        prevY = drivetrain.y;
        prevTh = drivetrain.currentheading;
        prevTime = curTime;
        prevydot = ydot;
        prevxdot = xdot;
        prevW = w;

        // Telemetry
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
        return new Waypoint(drivetrain.x, drivetrain.y, drivetrain.currentheading, xdot, ydot, xdotdot, ydotdot, 0);
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
        double[] ycoords = {r * Math.sin(pi/4 + theta) + y, r * Math.sin(3 * pi/4 + theta) + y, r * Math.sin(5 * pi/4 + theta) + y, r * Math.sin(7 * pi/4 + theta) + y};
        double[] xcoords = {r * Math.cos(pi/4 + theta) + x, r * Math.cos(3 * pi/4 + theta) + x, r * Math.cos(5 * pi/4 + theta) + x, r * Math.cos(7 * pi/4 + theta) + x};
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