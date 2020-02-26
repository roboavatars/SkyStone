package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("FieldCanBeLocal")
public class MecanumDrivetrain {

    //1440 ticks per encoder revolution
    //4.3289 inches per wheel revolution
    double encoderCountsPerRevolution = 537.6;

    //Motors of the drivetrain
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;
    private ModernRoboticsAnalogOpticalDistanceSensor stoneSensor;

    //copy of opmode related objects
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    //Objects for IMU and Rev Hub
    private LynxEmbeddedIMU imu;
    private LynxModule module;

    //IMU related variables for storing states
    private Orientation angles;
    private double lastheading = 0;
    private double deltaheading = 0;
    public double currentheading = 0;

    //Tracking x y coordinate position
    public double x=0;
    public double y=0;
    private double lastpod1 = 0;
    private double lastpod2 = 0;
    private double lastpod3 = 0;

    //k variables for control of linear system
    private double xk = 0.1;
    private double yk = 0.1;
    private double thetak = 0.8;


    public double deltapod1;
    public double deltapod2;
    public double deltapod3;


    private final double xyTolerance = 1;
    private final double thetaTolerance = Math.PI/35;
    private double OdometryTrackWidth = 13.74;
    private double OdometryHorizontalOffset = 3.17;
    private final double OdometryHeadingThreshold = Math.PI/8;

    public double lastx = 0;
    public double lasty = 0;
    public boolean stoneInRobot = false;
    public double distance;

    private boolean isRed;

    //motor caching stuff
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;

    public static double motorUpdateTolerance = 0.05;

    //Constructor
    public MecanumDrivetrain(LinearOpMode opMode, double initialx, double initialy, double initialtheta, boolean isRedAuto) {

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        module = hardwareMap.get(LynxModule.class,"Drivetrain Hub");

        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");
        stoneSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "stoneSensor");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new LynxEmbeddedIMU(new MecanumDrivetrain.BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));

        imu.initialize(new BNO055IMU.Parameters());

        x = initialx;
        y = initialy;
        lastheading = initialtheta;
        currentheading = initialtheta;

        isRed = isRedAuto;

        opMode.telemetry.addLine("ExH Version: " + getConciseLynxFirmwareVersion(module));
        opMode.telemetry.update();

    }

    private class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private static String getConciseLynxFirmwareVersion(LynxModule module) {
        String rawVersion = module.getFirmwareVersionString();
        String[] parts = rawVersion.split(" ");
        StringBuilder versionBuilder = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            String part = parts[3 + 2*i];
            if (i == 2) {
                versionBuilder.append(part);
            } else {
                versionBuilder.append(part, 0, part.length() - 1);
                versionBuilder.append(".");
            }
        }
        return versionBuilder.toString();
    }

    public void setControls(double xdot, double ydot, double w){
        double FRpower = ydot+xdot+w;
        double BLpower = ydot+xdot-w;
        double FLpower = -ydot+xdot-w;
        double BRpower = -ydot+xdot+w;

        double maxpower = Math.max(Math.abs(FRpower),Math.max(Math.abs(BLpower),
                Math.max(Math.abs(FLpower),Math.abs(BRpower))));

        if(maxpower > 1){
            FRpower /= maxpower;
            BLpower /= maxpower;
            FLpower /= maxpower;
            BRpower /= maxpower;
        }


        if(Math.abs(FRpower-lastFRPower)>motorUpdateTolerance || Math.abs(FLpower-lastFLPower)>motorUpdateTolerance
            || Math.abs(BRpower-lastBRPower)>motorUpdateTolerance || Math.abs(BLpower-lastBLPower)>motorUpdateTolerance){

            //set motor powers
            motorFrontRight.setPower(FRpower);
            motorBackLeft.setPower(BLpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);

            //cache new motor powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;
        }
    }

    public void setTargetPoint(double xtarget, double ytarget, double thetatarget){
        //make sure thetatarget is between 0 and 2pi
        thetatarget = thetatarget%(Math.PI*2);
        if(thetatarget<0){
            thetatarget += Math.PI*2;
        }
        //picking the smaller distance to rotate
        double thetacontrol = 0;
        if(currentheading-thetatarget>Math.PI){
            thetacontrol = currentheading-thetatarget-2*Math.PI;

        }
        else if(currentheading-thetatarget<(-Math.PI)){
            thetacontrol = currentheading-thetatarget+2*Math.PI;

        }
        else{
            thetacontrol = currentheading-thetatarget;
        }
        Log.w("auto", "thetacontrol: " + thetacontrol);

        setGlobalControls(-xk*(x-xtarget),-yk*(y-ytarget),-thetak*(thetacontrol));
    }

    public void setTargetPoint(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK){
        //make sure thetatarget is between 0 and 2pi
        thetatarget = thetatarget%(Math.PI*2);
        if(thetatarget<0){
            thetatarget += Math.PI*2;
        }
        //picking the smaller distance to rotate
        double thetacontrol = 0;
        if(Math.abs(currentheading-thetatarget)>Math.PI){
            thetacontrol = currentheading-thetatarget-2*Math.PI;
        }else{
            thetacontrol = currentheading-thetatarget;
        }

        setGlobalControls(-xK*(x-xtarget),-yK*(y-ytarget),-thetaK*(thetacontrol));
    }

    public void setTargetPointAuto(double xtarget, double ytarget, double thetatarget){
        if (!isRed) {
            xtarget = 144 - xtarget;
            thetatarget = (Math.PI) - thetatarget;
        }
        //make sure thetatarget is between 0 and 2pi
        thetatarget = thetatarget%(Math.PI*2);
        if(thetatarget<0){
            thetatarget += Math.PI*2;
        }
        //picking the smaller distance to rotate
        double thetacontrol = 0;
        if(Math.abs(currentheading-thetatarget)>Math.PI){
            thetacontrol = currentheading-thetatarget-2*Math.PI;
        }else{
            thetacontrol = currentheading-thetatarget;
        }

        Log.w("auto", "Targets: " + xtarget + " " + ytarget + " " + thetatarget + ", Current Pos: " + x + " " + y + " " + currentheading);
        setGlobalControls(-xk*(x-xtarget),-yk*(y-ytarget),-thetak*(thetacontrol));
    }

    public void setTargetPointAuto(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK) {
        if (!isRed) {
            xtarget = 144 - xtarget;
            thetatarget = (Math.PI) - thetatarget;
        }
        //make sure thetatarget is between 0 and 2pi
        thetatarget = thetatarget%(Math.PI*2);
        if(thetatarget<0){
            thetatarget += Math.PI*2;
        }
        //picking the smaller distance to rotate
        double thetacontrol = 0;
        if(Math.abs(currentheading-thetatarget)>Math.PI){
            thetacontrol = currentheading-thetatarget-2*Math.PI;
        }else{
            thetacontrol = currentheading-thetatarget;
        }

        Log.w("auto", "Targets: " + xtarget + " " + ytarget + " " + thetatarget + " (" + xK + " " + yK + " " + thetaK + "), Current Pos: " + x + " " + y + " " + currentheading);
        setGlobalControls(-xK * (x - xtarget), -yK * (y - ytarget), -thetaK * (thetacontrol));
    }

    public void setGlobalControls(double xvelocity, double yvelocity, double w){
        double xdot = xvelocity*Math.cos(-currentheading) - yvelocity*Math.sin(-currentheading);
        double ydot =  yvelocity*Math.cos(-currentheading) + xvelocity*Math.sin(-currentheading);
        setControls(xdot, ydot, w);

    }

    public LynxGetBulkInputDataResponse RevBulkData(){
        LynxGetBulkInputDataResponse response;
        try {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
            response = command.sendReceive();
        }
        catch (Exception e) {
            opMode.telemetry.addData("Exception", "bulk read exception");
            response = null;
        }
        return response;
    }

    public void updatePose(){
        try {
            LynxGetBulkInputDataResponse response = RevBulkData();
            double pod1 = -response.getEncoder(1) * 0.00300622055 * 2;
            double pod2 = response.getEncoder(0) * 0.00300622055 * 2;
            double pod3 = -response.getEncoder(2) * 0.00300622055 * 2;

            distance = response.getAnalogInput(0);
            stoneInRobot = distance > 150;

            deltapod1 = pod1 - lastpod1;
            deltapod2 = pod2 - lastpod2;
            deltapod3 = pod3 - lastpod3;

            lastx = x;
            lasty = y;

            deltaheading = (deltapod1 - deltapod2) / OdometryTrackWidth;

            double localx = (deltapod1 + deltapod2) / 2;
            double localy = deltapod3 - deltaheading * OdometryHorizontalOffset;

            if (deltaheading < OdometryHeadingThreshold) {
                x += localx * Math.cos(currentheading) - localy * Math.sin(currentheading);
                y += localy * Math.cos(currentheading) + localx * Math.sin(currentheading);

            } else {

                x += (localx * Math.sin(currentheading + deltaheading)
                        + localy * Math.cos(currentheading + deltaheading) - localx * Math.sin(currentheading)
                        - localy * Math.cos(currentheading)) / deltaheading;
                y += (localy * Math.sin(currentheading + deltaheading)
                        - localx * Math.cos(currentheading + deltaheading) - localy * Math.sin(currentheading) +
                        localx * Math.cos(currentheading)) / deltaheading;
            }
            currentheading += deltaheading;
            currentheading = currentheading%(Math.PI*2);
            if(currentheading<0){
                currentheading += Math.PI*2;
            }

            lastpod1 = pod1;
            lastpod2 = pod2;
            lastpod3 = pod3;
        } catch (Exception e) {e.printStackTrace();}

    }

    public double getHeadingImu(){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaheading = angles.firstAngle - lastheading;

        //opMode.telemetry.addData("delta", deltaheading);

        if (deltaheading < -Math.PI)
            deltaheading += 2*Math.PI ;
        else if (deltaheading >= Math.PI)
            deltaheading -= 2*Math.PI ;

        currentheading += deltaheading;

        lastheading = angles.firstAngle;

        return currentheading;

    }

    public void resetHeadingIMU(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastheading = angles.firstAngle;
        currentheading = 0;
    }

    public boolean isAtPoseAuto(double targetx, double targety, double targettheta) {

        return isAtPoseAuto(targetx, targety, targettheta, xyTolerance, xyTolerance, thetaTolerance);
    }
    public boolean isAtPoseAuto(double targetx, double targety, double targettheta, double xtolerance, double ytolerance, double thetatolerance) {
        if (!isRed) {
            targetx = 144 - targetx;
            targettheta = Math.PI - targettheta;
        }
        return (Math.abs(x - targetx) < xtolerance && Math.abs(y - targety) < ytolerance
                && Math.abs(currentheading - targettheta) < thetatolerance);
    }

}