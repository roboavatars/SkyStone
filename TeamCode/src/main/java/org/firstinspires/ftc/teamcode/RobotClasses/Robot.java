package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;

@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    private boolean firstLoop = true;

    // Subsystems
    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Stacker stacker;
    public FoundationGrabber grabber;
    public CapstoneDeposit capstoneDeposit;
    public Logger logger;

    private double prevX, prevY, prevTh, xdot, ydot, w, prevxdot = 0, prevydot = 0, prevTime, xdotdot, ydotdot;
    public double startTime;

    private boolean isAuto;

    // State booleans
    public boolean stoneInRobot = false;
    private boolean tryingToDeposit = false;
    private boolean downStacked = false;
    public boolean letGo = false;
    private boolean liftedUp = false;
    //private boolean depositAuto = false;

    // Class constants
    private final int stoneSensorUpdatePeriod = 7;
    private final int stoneValidationDistance = 3;
    private final int armTicksUpdatePeriod = 5;
    private final int loggerUpdatePeriod = 2;
    private final double AlignDistance = 5.7;

    private int cycleCounter = 0;
    public boolean isAutoAlign = false;
    public boolean intakeManual = false;
    public boolean isManualAlign = false;

    private LinearOpMode op;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta, boolean isAuto, boolean isRed) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta, isRed);
        intake = new Intake(op);
        stacker = new Stacker(op);
        grabber = new FoundationGrabber(op);
        capstoneDeposit = new CapstoneDeposit(op);
        logger = new Logger();

        this.op = op;
        this.isAuto = isAuto;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

    }

    public void update() {

        // increase cycle count
        cycleCounter++;

        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }

        if (!isAuto) {
            // check states-------------------------------------------------------
            if (!stoneInRobot && !tryingToDeposit) {
                stacker.goHome();
                stacker.unClampStone();
                if (!intakeManual) {
                    intake.setControls(0.7);
                }
            } else if (stacker.isArmOut() && stoneInRobot /*&& !depositAuto*/) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                //depositAuto = false;
                stacker.goHome();
                stacker.unClampStone();
            } else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit) {
                stacker.clampStone();
                if (!intakeManual) {
                    intake.setControls(0);
                }
            }
            // check if arm should lower to clamp stone
            else if (stoneInRobot && !tryingToDeposit && !stacker.isArmOut()) {
                stacker.goDown();
            } else if (stoneInRobot && !tryingToDeposit) {
                if (!intakeManual) {
                    intake.setControls(0);
                }
            }
            //check if we should downstack
            else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && !stacker.isDownStacked() && !downStacked && letGo) {
                stacker.downStack();
                downStacked = true;
            } else if (tryingToDeposit && stacker.isArmOut() && !stacker.isArmMoving() && stacker.isDownStacked() && letGo) {
                stacker.unClampStone();
                stacker.liftUp();
                liftedUp = true;
            } else if (tryingToDeposit && stacker.isArmOut() && stacker.isLiftUp() && downStacked && letGo && liftedUp) {
                tryingToDeposit = false;
                downStacked = false;
                letGo = false;
                //depositAuto = false;
                liftedUp = false;
                stacker.nextLevel();
            } else if (tryingToDeposit && !downStacked) {
                stacker.deposit();
                if (stacker.currentStackHeight > 0) {
                    grabber.extendRangeSensor();
                }
            }

            if (tryingToDeposit && (!letGo /*|| depositAuto*/) && cycleCounter % 2 == 0 && stacker.currentStackHeight > 0 && !isManualAlign) {
                grabber.extendRangeSensor();
                double distance = grabber.getDistance();
                op.telemetry.addData("align dist", distance);
                if (Math.abs(distance - AlignDistance) < 7) {
                    drivetrain.setControls(-0.25 * (distance - AlignDistance), -0.15, 0);
                } else {
                    drivetrain.setControls(0, 0, 0);
                }
                isAutoAlign = true;

//                if (depositAuto) {
//                    if (Math.abs(distance - AlignDistance) < 0.5 || Math.abs(distance - AlignDistance) > 12) {
//                        letGo = true;
//                    }
//                }
            } else if (cycleCounter % 2 == 0) {
                isAutoAlign = false;
            }
        }
        else {
            if(!tryingToDeposit && !stoneInRobot){
                stacker.goHome();
            }
            else if (stoneInRobot && stacker.isArmHome() && !tryingToDeposit) {
                stacker.goDown();
            }
            else if (stoneInRobot && stacker.isArmDown() && !tryingToDeposit) {
                stacker.clampStone();
                intake.setControls(0);
            }
            else if (stoneInRobot && tryingToDeposit && !stacker.atautodepositpos()) {
                stacker.depositAuto();
            }
            //TODO fix this hardcoded postion
            else if (!stoneInRobot && tryingToDeposit && stacker.getArmPosition()>850) {
                stacker.unClampStone();
                tryingToDeposit = false;
                intake.setControls(0.7);
            }
        }

        drivetrain.updatePose();
        stoneInRobot = drivetrain.stoneInRobot;

//        if (cycleCounter % loggerUpdatePeriod == 0) {
//            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,velocityX,velocityY,velocityTh,stoneInRobot,stacker.stoneClamped,tryingToDeposit,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
//        }

        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        //calculating velocity/acceleration
        xdot = (drivetrain.x - prevX) / timeDiff;
        ydot = (drivetrain.y - prevY) / timeDiff;
        w = (drivetrain.currentheading - prevTh) / timeDiff;
        xdotdot = (xdot-prevxdot)/timeDiff;
        ydotdot = (ydot-prevydot)/timeDiff;

        //setting tracking variables for old states
        prevX = drivetrain.x;
        prevY = drivetrain.y;
        prevTh = drivetrain.currentheading;
        prevTime = curTime;
        prevydot = ydot;
        prevxdot = xdot;

        drawRobot(drivetrain.x, drivetrain.y, drivetrain.currentheading);
        addPacket("X", drivetrain.x);
        addPacket("Y", drivetrain.y);
        addPacket("Theta", drivetrain.currentheading);
        addPacket("is stone in robot", stoneInRobot);
        addPacket("distance", drivetrain.distance);
        addPacket("robot velocity", Math.sqrt(Math.pow(xdot,2) + Math.pow(ydot, 2)));
//        addPacket("arm", stacker.isArmHome() + " " + stacker.isArmDown() + " " + stacker.isArmOut() + " " );
        addPacket("loop time", timeDiff);
        addPacket("update frequency(hz)", 1/timeDiff);
        sendPacket();
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
        packet.fieldOverlay().setFill("green").fillPolygon(xcoords,ycoords);
    }

    public void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public void deposit() {
        if (!stacker.isArmOut()) {
            tryingToDeposit = true;
        }
    }

    public void depositAuto() {
        if (!stacker.isArmOut() && stoneInRobot) {
            tryingToDeposit = true;
            //depositAuto = true;
        }
    }
    public void log(String message) {
        Log.w("robot", " ");
        Log.w("robot", message + " -------------------------------------------------------------------------");
    }
    public Waypoint currentRobotWaypoint(){
        return new Waypoint(drivetrain.x, drivetrain.y, drivetrain.currentheading,
                xdot, ydot, xdotdot, ydotdot, 0);
    }

//    public void setSafeControls(double x, double y, double theta) {
//        drivetrain.setTargetPoint();
//    }
}
