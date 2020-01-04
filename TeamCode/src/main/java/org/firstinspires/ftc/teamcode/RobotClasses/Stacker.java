package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Stacker {
    
    //Electronics
    private LynxModule module;
    private DcMotorEx liftMotor;
    private DcMotorEx depositMotor;
    private Servo stoneClamp;

    public int basepos = 0;
    
    private final double clampPos = 0.4;
    private final double unClampPos = 0.99;
    
    public final int armPos[] =   {665, 665, 665, 665, 301, 301, 301, 301, 301, 301}; // increases down
    private final int liftPos[] = {-540, -992, -1444, -1896, 0, -452,-904, -1356, -1808, -2260};

    private int currentStackHeight = 0;
    private int armTicks = 0;
    private int liftTicks = 0;

    public boolean stoneClamped = false;
    private final int armOut = 250;
    private final int armDown = -10;
    private final int armHome = 25;
    private final int armTolerance = 10;
    private final int liftHome = 0;
    private final int liftTolerance = 10;
    private final int moveliftUpHeight = 100;
    //unit is ticks/second

    private double armVelocity = 0;
    private double liftVelocity = 0;
    private final int armVelocityTolerance = 1;
    private final int liftVelocityTolerance = 1;
    
    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;
    
    public Stacker(LinearOpMode op) {
        
        this.op = op;
        this.hardwareMap = op.hardwareMap;

        module = hardwareMap.get(LynxModule.class,"Other Stuff");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        depositMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        stoneClamp = hardwareMap.get(Servo.class, "stoneClamp");
        
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        depositMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setVelocityPIDFCoefficients(2,0.5,0, 20);


        depositMotor.setTargetPosition(0);
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setTargetPositionTolerance(0);
        depositMotor.setPositionPIDFCoefficients(15);
        depositMotor.setVelocityPIDFCoefficients(2,0.3,0,20);
        op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

        liftMotor.setPower(0);
        depositMotor.setPower(0);

        op.telemetry.addData("Status", "Stacker Initialized");

    }

    public LynxGetBulkInputDataResponse RevBulkData(){
        LynxGetBulkInputDataResponse response;
        try {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
            response = command.sendReceive();
        }
        catch (Exception e) {
            op.telemetry.addData("Exception", "bulk read exception");
            response = null;
        }
        return response;
    }
    
    public void setLiftControls(double power, int ticks) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(ticks);
    }
    public void setDepositControls(double power, int ticks) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setTargetPosition(ticks);
        depositMotor.setPower(power);
    }

    public void goHome() {
        if(armTicks<20){
            setDepositControls(1,armHome);
        }
        else {
            setDepositControls(0.25, armHome);
        }
        setLiftControls(0.2,0);

    }
    public void goDown() {
        setDepositControls(1.0, armDown);
    }
    public void downStack() {
        setLiftControls(0.75,liftPos[currentStackHeight]);
    }

    public void deposit() {
        setLiftControls(1.0, liftPos[currentStackHeight]-300);
        setDepositControls(0.3, armPos[currentStackHeight]);
    }
    public void liftUp(){
        setLiftControls(1,liftPos[currentStackHeight]-moveliftUpHeight);
    }

    public boolean isArmHome() {

        return Math.abs(getArmPosition() - armHome) < armTolerance;
    }
    public boolean isLiftHome() {
        return Math.abs(getLiftPosition() - liftHome) < liftTolerance;
    }
    public boolean isLiftUp() {
        return Math.abs(getLiftPosition() - (liftPos[currentStackHeight]-moveliftUpHeight)) < 40;
    }
    public boolean isArmOut() {

        return getArmPosition() > armOut;
    }
    public boolean isArmDown() {

        return Math.abs(getArmPosition()+10) < armTolerance && !isArmMoving();
    }

    public void nextLevel() {
        currentStackHeight = Math.min(currentStackHeight + 1, 9);
    }
    public void lastLevel() {
        currentStackHeight = Math.max(currentStackHeight - 1, 0);
    }

    public void setLevel(int level) {
        currentStackHeight = level;
    }
    
    public void clampStone() {
        stoneClamp.setPosition(clampPos);
        stoneClamped = true;
    }
    public void unClampStone() {
        stoneClamp.setPosition(unClampPos);
        stoneClamped = false;
    }
    
    public int getLiftPosition() {
        return liftTicks;
    }
    public int getArmPosition() {
        return armTicks;
    }

    public boolean isArmMoving() {
        return Math.abs(armVelocity) > armVelocityTolerance;
    }
    public boolean isLiftMoving(){
        return Math.abs(liftVelocity) > liftVelocityTolerance;
    }
    public boolean isDownStacked(){
        return Math.abs(liftTicks - (liftPos[currentStackHeight])) < 40 && !isLiftMoving();
    }

    public void update() {
        LynxGetBulkInputDataResponse response = RevBulkData();

        armTicks = response.getEncoder(2);
        liftTicks = response.getEncoder(3);
        armVelocity = response.getVelocity(2);
        liftVelocity = response.getVelocity(3);
//        setArmPower(Math.signum(armTargetPos-armTicks)*Math.max(0.0075*(Math.abs(armTicks-armTargetPos))*armPower, 0.15*Math.cos(getArmAngle())) + 0.35*Math.cos(getArmAngle()));

    }

    public void setArmPower(double p){
        depositMotor.setPower(p);
    }
    public double getArmAngle(){
        return 2*Math.PI*(armTicks-69.0)/806.4;
    }
}
