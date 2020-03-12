package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
//@Config
public class Stacker {
    
    // Electronics
    private LynxModule module;
    private DcMotorEx liftMotor;
    private DcMotorEx depositMotor;
    private Servo stoneClamp;

    // Clamp Positions
    private final double clampPos = 0.15;
    private final double unClampPos = 0.9;

    public boolean stoneClamped = false;

    // Arm/Lift Deposit Positions
    // The Lower the Pos Value, The Higher the Arm
    private final int[] armPos = {1320, 1320, 1320, 1320, 635, 635, 635, 635, 635, 635, 635, 635, 635};
    private final int[] liftPos = {-540, -992, -1444, -1896, 0, -600, -1050, -1430, -1950, -2380, -2910, -3490, -4170};
    private final int[] liftMin = {-100, -250, -600, -700, 0, -400, -850, -1100, -1600, -1950, -2480, -3160, -3740}; // min lift ticks to raise arm
    private int autoDepositPos = 950;

    public int currentStackHeight = 0;
    private int armTicks = 0;
    private int liftTicks = 0;

    // Encoder Positions
    private final int armOut = 500;
    private final int armDown = -30;
    private final int armHome = 35;
    private final int armIntermediatePos = 180;
    private final int armTolerance = 25;
    private final int liftHome = 100;
    private final int liftTolerance = 10;
    private final int moveLiftUpHeight = 400;

    public int manualArmOffset = 0;
    public int manualLiftOffset = 0;

    // Velocity Variables (Ticks/Second)
    private double armVelocity = 0;
    private double liftVelocity = 0;
    private final int armVelocityTolerance = 2;
    private final int liftVelocityTolerance = 5;

    // OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    // Caching Stuff
    private int armLastTargetPos = 0;
    private double armLastTargetPower = 0;
    private int liftLastTargetPos = 0;
    private double liftLastTargetPower = 0;
    private boolean setLiftPID = false;

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
        liftMotor.setTargetPositionTolerance(5);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setVelocityPIDFCoefficients(2,0.5,0, 5);
        liftMotor.setPositionPIDFCoefficients(18);
        //op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

        depositMotor.setTargetPosition(0);
        depositMotor.setTargetPositionTolerance(0);
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setVelocityPIDFCoefficients(2,0.3,0,20);
        depositMotor.setPositionPIDFCoefficients(18);
        //op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());

        liftMotor.setPower(0);
        depositMotor.setPower(0);
        stoneClamp.setPosition(unClampPos);

        op.telemetry.addData("Status", "Stacker Initialized");
    }

    // Basic Arm Movement
    public void setDepositControls(double power, int ticks) {
        if (power != armLastTargetPower) {
            depositMotor.setPower(power);
            armLastTargetPower = power;
        }
        if (ticks != armLastTargetPos) {
            depositMotor.setTargetPosition(-ticks);
            armLastTargetPos = ticks;
        }
    }

    // Basic Lift Movement
    public void setLiftControls(double power, int ticks) {
        if (power != liftLastTargetPower) {
            liftMotor.setPower(power);
            liftLastTargetPower = power;
        }
        if (ticks != liftLastTargetPos) {
            liftMotor.setTargetPosition(ticks);
            liftLastTargetPos = ticks;
        }
    }

    // Set Arm Position Methods
    public void goHome() {
        if (armTicks < 40) {
            setDepositControls(1, armHome);
        } else {
            setDepositControls(0.5, armHome);
        }
//        if (liftTicks > -50 && !setLiftPID) {
//            liftMotor.setPositionPIDFCoefficients(18);
//            setLiftPID = true;
//        } else if (setLiftPID && liftTicks <- 50) {
//            liftMotor.setPositionPIDFCoefficients(5);
//            setLiftPID = false;
//        }
        setLiftControls(1.0, liftHome);
    }
    public void goDown() {
        setDepositControls(1.0, armDown);
    }

    // Set Lift Position Methods
    public void downStack() {
        setLiftControls(1.0, liftPos[currentStackHeight] + manualLiftOffset);
    }
    public void liftUp(){
        setLiftControls(1, liftPos[currentStackHeight] - moveLiftUpHeight + manualLiftOffset);
    }

    // Depositing Methods
    public void deposit() {
        if (liftMin[currentStackHeight] > liftTicks) {
            setDepositControls(0.48, armPos[currentStackHeight] + manualArmOffset);
        } else {
            setDepositControls(0.48, armIntermediatePos);
        }
        setLiftControls(1.0, liftPos[currentStackHeight] - 300 + manualLiftOffset);
    }
    public void depositAuto() {
        setDepositControls(0.8, autoDepositPos);
    }

    // Arm State Methods
    public boolean isArmHome() {
        return Math.abs(getArmPosition() - armHome-10) < armTolerance;
    }
    public boolean isArmOut() {
        return getArmPosition() > armOut;
    }
    public boolean isArmDown() {
        return Math.abs(getArmPosition() + 20) < armTolerance && !isArmMoving();
    }
    public boolean atAutoDepositPos() {
        return Math.abs(getArmPosition() - autoDepositPos) < armTolerance;
    }
    public boolean isArmMoving() {
        return Math.abs(armVelocity) > armVelocityTolerance;
    }

    // Lift State Methods
    public boolean isLiftHome() {
        return Math.abs(getLiftPosition() - liftHome) < liftTolerance;
    }
    public boolean isDownStacked() {
        return Math.abs(liftTicks - (liftPos[currentStackHeight]) + manualLiftOffset) < 100 && !isLiftMoving();
    }
    public boolean isLiftUp() {
        return Math.abs(getLiftPosition() - (liftPos[currentStackHeight] - moveLiftUpHeight + manualLiftOffset)) < 40 && !isLiftMoving();
    }
    public boolean isLiftMoving() {
        return Math.abs(liftVelocity) > liftVelocityTolerance;
    }

    // Stack Level Methods
    public void nextLevel() {
        currentStackHeight = Math.min(currentStackHeight + 1, 13);
    }
    public void lastLevel() {
        currentStackHeight = Math.max(currentStackHeight - 1, 0);
    }
    public void setLevel(int level) {
        currentStackHeight = level;
    }

    // Stone Clamp/Unclamp Methods
    public void clampStone() {
//        if (!stoneClamped) {
//            stoneClamp.setPosition(clampPos);
//            stoneClamped = true;
//        }
        stoneClamp.setPosition(clampPos);
        stoneClamped = true;

    }
    public void unClampStone() {
//        if (stoneClamped) {
//            stoneClamp.setPosition(unClampPos);
//            stoneClamped = false;
//        }
        stoneClamp.setPosition(unClampPos);
        stoneClamped = false;
    }

    // Encoder Tick Methods
    public int getArmPosition() {
        return armTicks;
    }
    public int getLiftPosition() {
        return liftTicks;
    }

    // Get Angle of Arm
    public double getArmAngle() {
        return 2 * Math.PI * (armTicks / 2 - 69.0) / 806.4;
    }

    // Arm/Lift Bulk Read Methods
    public LynxGetBulkInputDataResponse RevBulkData(){
        LynxGetBulkInputDataResponse response;
        try {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
            response = command.sendReceive();
        } catch (Exception e) {
            op.telemetry.addData("Exception", "bulk read exception");
            response = null;
        }
        return response;
    }

    public void update() {
        LynxGetBulkInputDataResponse response = RevBulkData();

        armTicks = response.getEncoder(2);
        liftTicks = response.getEncoder(3);
        armVelocity = response.getVelocity(2);
        liftVelocity = response.getVelocity(3);
    }
}