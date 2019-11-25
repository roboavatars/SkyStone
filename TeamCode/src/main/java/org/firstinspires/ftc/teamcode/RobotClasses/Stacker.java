package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Stacker {
    
    //Electronics
    private DcMotorEx liftMotor;
    private DcMotorEx depositMotor;
    private Servo stoneClamp;

    public int basepos = 0;
    
    private final double clampPos = 0.9;
    private final double unClampPos = 0.65;
    
    public final int armPos[] =   {2150, 1850, 1650, 1430, 1260, 1260, 1260, 1260, 1260, 1260}; // increases down
    private final int liftPos[] = {0,    0,    0,    0,    350,  840,  1360, 1830, 2300, 2760};

    private int currentStackHeight = 0;
    private int armTicks = 0;
    private int liftTicks = 0;
    private double armVelocity = 0;
    public boolean stoneClamped = false;
    private final int armOut = 1100;
    private final int armDown = 25;
    private final int armHome = 180;
    private final int armTolerance = 25;
    private final int liftHome = 0;
    private final int liftTolerance = 10;
    //unit is ticks/second
    private final int armVelocityTolerance = 5;
    
    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;
    
    public Stacker(LinearOpMode op) {
        
        this.op = op;
        this.hardwareMap = op.hardwareMap;
        
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        depositMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        stoneClamp = hardwareMap.get(Servo.class, "stoneClamp");
        
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        depositMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        depositMotor.setTargetPosition(0);
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        liftMotor.setPower(0);
        depositMotor.setPower(0);
        
        op.telemetry.addData("Status", "Stacker Initialized");
        op.telemetry.update();
    }
    
    public void setLiftControls(double power, int ticks) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(ticks);
    }
    public void setDepositControls(double power, int ticks) {
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setPower(power);
        depositMotor.setTargetPosition(Math.abs(ticks + basepos));
    }

    public void goHome() {
        setDepositControls(0.5,armHome);
        setLiftControls(0.5,0);
    }
    public void goDown() {
        setDepositControls(0.3, armDown);
    }
    public void downStack() {
        setDepositControls(0.3,armTicks + 130);
    }

    public void deposit() {
        setLiftControls(1, liftPos[currentStackHeight]);
        setDepositControls(0.75, armPos[currentStackHeight]);
    }

    public boolean isArmHome() {
        return Math.abs(getArmPosition() - armHome) < armTolerance;
    }
    public boolean isLiftHome() {
        return Math.abs(getLiftPosition() - liftHome) < liftTolerance;
    }
    public boolean isArmOut() {
        return getArmPosition() > armOut;
    }
    public boolean isArmDown() {
        return Math.abs(getArmPosition() - armDown) < armTolerance;
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

    public void update() {
        armTicks = depositMotor.getCurrentPosition();
        liftTicks = liftMotor.getCurrentPosition();
        armVelocity = depositMotor.getVelocity();
    }
}
