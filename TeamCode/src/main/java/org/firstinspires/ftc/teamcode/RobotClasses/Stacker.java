package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Stacker {
    
    // electronics
    private LynxModule module;
    private DcMotorEx liftMotor;
    private DcMotorEx depositMotor;
    private Servo stoneClamp;

    // clamp positions
    private final double clampPos = 0.01;
    private final double unClampPos = 0.7;

    public boolean stoneClamped = false;

    // arm/lift deposit positions
    // the lower the pos value, the higher the arm
    private final int[] armPos = {1320, 1320, 1320, 1320, 635, 635, 635, 635, 635, 635};
    private final int[] liftPos = {-640, -1092, -1544, -1996, 0, -500, -1050, -1430, -1950, -2530};
    private final int[] liftMin = {0, 0, 0, -500, 0, 0, -500, -900, -1400, -1800};
    private int autoDepositPos = 950;

    public int currentStackHeight = 0;
    private int armTicks = 0;
    private int liftTicks = 0;

    // encoder positions
    private final int armOut = 500;
    private final int armDown = -30;
    private final int armHome = 27;
    private final int armIntermediatePos = 180;
    private final int armTolerance = 25;
    private final int liftHome = 20;
    private final int liftTolerance = 10;
    private final int moveLiftUpHeight = 400;

    public int manualArmOffset = 0;

    // velocity variables- unit is ticks/second
    private double armVelocity = 0;
    private double liftVelocity = 0;
    private final int armVelocityTolerance = 2;
    private final int liftVelocityTolerance = 5;

    // opmode stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    // caching stuff
    private int armLastTargetPos = 0;
    private double armLastTargetPower = 0;
    private int liftLastTargetPos = 0;
    private double liftLastTargetPower = 0;

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
        liftMotor.setTargetPositionTolerance(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setVelocityPIDFCoefficients(2,0.5,0, 15);
        liftMotor.setPositionPIDFCoefficients(18);
        //op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

        depositMotor.setTargetPosition(0);
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setTargetPositionTolerance(0);
        depositMotor.setPositionPIDFCoefficients(18);
        depositMotor.setVelocityPIDFCoefficients(2,0.3,0,20);
        //op.telemetry.addLine(depositMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());

        liftMotor.setPower(0);
        depositMotor.setPower(0);
        stoneClamp.setPosition(unClampPos);

        op.telemetry.addData("Status", "Stacker Initialized");
    }

    // basic arm movement
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

    // basic lift movement
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

    // set arm position methods
    public void goHome() {
        if (armTicks < 40) {
            setDepositControls(1, armHome);
        } else {
            setDepositControls(0.5, armHome);
        }
//        if (liftTicks > -50) {
//            liftMotor.setPositionPIDFCoefficients(18);
//        } else {
//            liftMotor.setPositionPIDFCoefficients(5);
//        }
        setLiftControls(1.0, liftHome);
    }
    public void goDown() {
        setDepositControls(1.0, armDown);
    }

    // set lift position methods
    public void downStack() {
        setLiftControls(1.0, liftPos[currentStackHeight]);
    }
    public void liftUp(){
        setLiftControls(1, liftPos[currentStackHeight] - moveLiftUpHeight);
    }

    // depositing methods
    public void deposit() {
        if (liftMin[currentStackHeight] > liftTicks) {
            setDepositControls(0.44, armPos[currentStackHeight] + manualArmOffset);
        } else {
            setDepositControls(0.44, armIntermediatePos);
        }
        setLiftControls(0.8, liftPos[currentStackHeight] - 300);
    }
    public void depositAuto() {
        setDepositControls(0.6, autoDepositPos);
    }

    // check arm state methods
    public boolean isArmHome() {
        return Math.abs(getArmPosition() - armHome) < armTolerance;
    }
    public boolean isArmOut() {
        return getArmPosition() > armOut;
    }
    public boolean isArmDown() {
        return Math.abs(getArmPosition() + 30) < armTolerance && !isArmMoving();
    }
    public boolean atAutoDepositPos() {
        return Math.abs(getArmPosition() - autoDepositPos) < armTolerance;
    }
    public boolean isArmMoving() {
        return Math.abs(armVelocity) > armVelocityTolerance;
    }

    // check lift state methods
    public boolean isLiftHome() {
        return Math.abs(getLiftPosition() - liftHome) < liftTolerance;
    }
    public boolean isDownStacked() {
        return Math.abs(liftTicks - (liftPos[currentStackHeight])) < 100 && !isLiftMoving();
    }
    public boolean isLiftUp() {
        return Math.abs(getLiftPosition() - (liftPos[currentStackHeight]- moveLiftUpHeight)) < 40 && !isLiftMoving();
    }
    public boolean isLiftMoving() {
        return Math.abs(liftVelocity) > liftVelocityTolerance;
    }

    // change stack level methods
    public void nextLevel() {
        currentStackHeight = Math.min(currentStackHeight + 1, 9);
    }
    public void lastLevel() {
        currentStackHeight = Math.max(currentStackHeight - 1, 0);
    }
    public void setLevel(int level) {
        currentStackHeight = level;
    }

    // clamp/unclamp stone methods
    public void clampStone() {
        if (!stoneClamped) {
            stoneClamp.setPosition(clampPos);
            stoneClamped = true;
        }
    }
    public void unClampStone() {
        if (stoneClamped) {
            stoneClamp.setPosition(unClampPos);
            stoneClamped = false;
        }
    }

    // get encoder ticks methods
    public int getArmPosition() {
        return armTicks;
    }
    public int getLiftPosition() {
        return liftTicks;
    }

    // get angle of arm
    public double getArmAngle() {
        return 2 * Math.PI * (armTicks / 2 - 69.0) / 806.4;
    }

    // arm/lift bulk read methods
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
