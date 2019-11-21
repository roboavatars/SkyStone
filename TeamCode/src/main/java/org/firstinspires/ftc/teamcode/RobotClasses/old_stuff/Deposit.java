package org.firstinspires.ftc.teamcode.RobotClasses.old_stuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
@Deprecated
public class Deposit {

    //Electronics
    private DcMotorEx depositMotor;
    private Servo depositServo;

    private final double openPos = 0;
    private final double closePos = 0.075;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Deposit(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        depositMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        depositServo = hardwareMap.get(Servo.class, "depositServo");

        depositMotor.setDirection(DcMotor.Direction.FORWARD);

        op.telemetry.addData("Status", "Deposit Initialized");
        op.telemetry.update();
    }

    public void setControls(double motorPower) {
        depositMotor.setPower(motorPower);
    }

    public void clampStone() {
        depositServo.setPosition(openPos);
    }

    public void unclampStone() {
        depositServo.setPosition(closePos);
    }
    
    public void extendDeposit() {
    
    }
    
    public void retractDeposit() {
    
    }

    public double getDepositPos() {
        return depositServo.getPosition();
    }
}
