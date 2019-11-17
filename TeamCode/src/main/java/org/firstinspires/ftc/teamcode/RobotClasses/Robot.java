package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public MecanumDrivetrain drivetrain;
    public Intake intake;
    public Deposit deposit;
    public Clamp clamp;
    public Rev2mDistanceSensor stoneSensor;

    public Robot(HardwareMap hardwareMap, LinearOpMode op, double initx, double inity, double inittheta) {
        drivetrain = new MecanumDrivetrain(hardwareMap, op, initx, inity, inittheta);
        intake = new Intake(hardwareMap, op);
        deposit = new Deposit(hardwareMap, op);
        clamp = new Clamp(hardwareMap, op);

    }
}
