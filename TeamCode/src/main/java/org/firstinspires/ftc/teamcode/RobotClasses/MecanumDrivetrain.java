package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
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
import org.opencv.core.Mat;

import java.util.Iterator;

/*
Methods in the class:
Changing motor power
Setting constant velocity
Lynx firmware stuff
Getting current position
Sets motors to RUN_USING_ENCODER
Gets angle
Resets angle
45 degree strafe (2 wheel)
Strafe in any direction (4 wheel)
 */

//1440 ticks per encoder revolution
//4.3289 inches per wheel revolution

public class MecanumDrivetrain {
    double encoderCountsPerRevolution = 537.6;
    //Motors of the drivetrain
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

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

    private final double xyTolerance = 1;
    private final double thetaTolerance = Math.PI/35;

    //Constructor
    public MecanumDrivetrain(LinearOpMode opMode, double initialx, double
                             initialy, double initialtheta){

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        module = hardwareMap.get(LynxModule.class,"Drivetrain Hub");

        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");

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

    public void setControls(double xvelocity, double yvelocity, double w){
        motorFrontRight.setPower(xvelocity+yvelocity+w);
        motorBackLeft.setPower(xvelocity+yvelocity-w);
        motorFrontLeft.setPower(-xvelocity+yvelocity-w);
        motorBackRight.setPower(-xvelocity+yvelocity+w);
    }
    public void setTargetPoint(double xtarget, double ytarget, double thetatarget){
        setGlobalControls(-xk*(x-xtarget),-yk*(y-ytarget),-thetak*(currentheading-thetatarget));
    }
    public void setTargetPoint(double xtarget, double ytarget, double thetatarget, double xK, double yK, double thetaK){
        setGlobalControls(-xK*(x-xtarget),-yK*(y-ytarget),-thetaK*(currentheading-thetatarget));
    }
    public void setGlobalControls(double xvelocity, double yvelocity, double w){

        double xdot = xvelocity*Math.cos(-currentheading) - yvelocity*Math.sin(-currentheading);
        double ydot =  yvelocity*Math.cos(-currentheading) + xvelocity*Math.sin(-currentheading);

        double FRpower = ydot+xdot+w;
        double BLpower = ydot+xdot-w;
        double FLpower = -ydot+xdot-w;
        double BRpower = -ydot+xdot+w;

        double maxpower = Math.max(Math.abs(FRpower),Math.max(Math.abs(BLpower),
                Math.max(Math.abs(FLpower),Math.abs(BRpower))));

        if(Math.abs(maxpower)>1){
            FRpower *= 1/maxpower;
            BLpower *= 1/maxpower;
            FLpower *= 1/maxpower;
            BRpower *= 1/maxpower;
        }

        motorFrontRight.setPower(FRpower);
        motorBackLeft.setPower(BLpower);
        motorFrontLeft.setPower(FLpower);
        motorBackRight.setPower(BRpower);
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
        LynxGetBulkInputDataResponse response = RevBulkData();
        double pod1 = -response.getEncoder(0)*0.00300622055*2;
        double pod2 = response.getEncoder(1)*0.00300622055*2;
        double pod3 = response.getEncoder(2)*0.00300622055*2;

        double deltapod1 = pod1 - lastpod1;
        double deltapod2 = pod2 - lastpod2;
        double deltapod3 = pod3 - lastpod3;


//        opMode.telemetry.addData("pod1 inches", pod1);
//        opMode.telemetry.addData("pod2 inches", pod2);
//        opMode.telemetry.addData("pod3 inches", pod3);

        deltaheading = (deltapod1-deltapod2)/13.74;

        double localx = (deltapod1+deltapod2)/2;
        double localy = deltapod3 - deltaheading*2.54;

        if(deltaheading<Math.PI/8){
            x += localx*Math.cos(currentheading) - localy*Math.sin(currentheading);
            y += localy*Math.cos(currentheading) + localx*Math.sin(currentheading);

        }
        else{

            x += (localx*Math.sin(currentheading+deltaheading)
                    +localy*Math.cos(currentheading+deltaheading) - localx*Math.sin(currentheading)
                    - localy*Math.cos(currentheading))/deltaheading;
            y += (localy*Math.sin(currentheading+deltaheading)
                    -localx*Math.cos(currentheading+deltaheading)-localy*Math.sin(currentheading)+
                    localx*Math.cos(currentheading))/deltaheading;
        }
        currentheading += deltaheading;

        lastpod1 = pod1;
        lastpod2 = pod2;
        lastpod3 = pod3;

    }

    public double getAngle(){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaheading = angles.firstAngle - lastheading;

//        opMode.telemetry.addData("delta", deltaheading);

        if (deltaheading < -Math.PI)
            deltaheading += 2*Math.PI ;
        else if (deltaheading >= Math.PI)
            deltaheading -= 2*Math.PI ;

        currentheading += deltaheading;

        lastheading = angles.firstAngle;

        return currentheading;

    }
    public void resetAngle(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastheading = angles.firstAngle;
        currentheading = 0;
    }


    public boolean isAtPose(double targetx, double targety, double targettheta) {
        return (Math.abs(x - targetx) < xyTolerance && Math.abs(y - targety) < xyTolerance
                && Math.abs(currentheading - targettheta) < thetaTolerance);
    }
    public boolean isAtPose(double targetx, double targety, double targettheta, double xtolerance, double ytolerance, double thetatolerance) {
        return (Math.abs(x - targetx) < xtolerance && Math.abs(y - targety) < ytolerance
                && Math.abs(currentheading - targettheta) < thetatolerance);
    }


}


