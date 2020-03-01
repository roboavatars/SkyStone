package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class CapstoneDeposit {

    private Servo capstoneDeposit;

    // opmode stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public CapstoneDeposit(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        capstoneDeposit = hardwareMap.get(Servo.class, "capstoneDeposit");
        goHome();
    }

    public void attachCapstone() {
        capstoneDeposit.setPosition(0.75);
    }

    public void goHome(){
        capstoneDeposit.setPosition(0.15);
    }

}
