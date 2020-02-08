package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

    private double prevX, prevY, prevTh, velocityX, velocityY, velocityTh, prevTime;
    public double startTime;

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
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public Robot(LinearOpMode op, double initX, double initY, double initTheta, boolean isRedAuto) {
        drivetrain = new MecanumDrivetrain(op, initX, initY, initTheta, isRedAuto);
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

        if (firstLoop) {
            startTime = System.currentTimeMillis();
            firstLoop = false;
        }

        // update arm
        if ((cycleCounter + 3) % armTicksUpdatePeriod == 0) {
            stacker.update();
        }
        if(depositAuto && stacker.currentStackHeight == 0){
            letGo = true;
        }

        // check states----------------------
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
        // check if arm should lower in preparation to clamp stone
        else if (stoneInRobot && !tryingToDeposit && !stacker.isArmOut()) {
            stacker.goDown();
        }
        else if(stoneInRobot && !tryingToDeposit){
            if(!intakeManual){
                intake.setControls(0);
            }
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

//        if (cycleCounter % loggerUpdatePeriod == 0) {
//            logger.logData(System.currentTimeMillis()-startTime,drivetrain.x,drivetrain.y,drivetrain.currentheading,velocityX,velocityY,velocityTh,stoneInRobot,stacker.stoneClamped,tryingToDeposit,stacker.isArmHome(),stacker.isArmDown(),stacker.isArmOut());
//        }
//        if (cycleCounter % flushUpdatePeriod == 0) logger.flush();

        double curTime = (double) System.currentTimeMillis() / 1000;
        double timeDiff = curTime - prevTime;
        velocityX = (drivetrain.x - prevX) / timeDiff;
        velocityY = (drivetrain.y - prevY) / timeDiff;
        velocityTh = (drivetrain.currentheading - prevTh) / timeDiff;
        prevX = drivetrain.x; prevY = drivetrain.y; prevTh = drivetrain.currentheading; prevTime = curTime;

        drawRobot(drivetrain.x, drivetrain.y, drivetrain.currentheading);
        addPacket("X", drivetrain.x);
        addPacket("Y", drivetrain.y);
        addPacket("Theta", drivetrain.currentheading);
        addPacket("is stone in robot", stoneInRobot);

//        addPacket("arm", stacker.isArmHome() + " " + stacker.isArmDown() + " " + stacker.isArmOut() + " " );
        addPacket("loop time", timeDiff);
        addPacket("update frequency(hz)", 1/timeDiff);

        sendPacket();
        packet = new TelemetryPacket();

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
