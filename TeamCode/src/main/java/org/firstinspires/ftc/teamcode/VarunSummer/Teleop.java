package org.firstinspires.ftc.teamcode.VarunSummer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Teleop extends LinearOpMode {

    // magic numbers/constants
    final double INTAKE_POWER = 1.0;
    final double SLIDES_POWER = 1.0;
    final int TARGET_POS = 500;
    final double INITIAL_DUMPER_POSITION = 0.0;
    final double DUMPED_POSITION = 1.0;

    //motors and servo
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private DcMotor intakeMotor;

    private DcMotor slidesMotor;
    private Servo dumperServo;

    @Override
    public void runOpMode() {
        // getting the hardware
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        slidesMotor = hardwareMap.dcMotor.get("slidesMotor");
        dumperServo = hardwareMap.servo.get("dumperServo");

        // initializing the motors and servo
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);

        intakeMotor.setPower(0.0);
        slidesMotor.setPower(0.0);

        dumperServo.setPosition(INITIAL_DUMPER_POSITION);

        waitForStart();

        while(opModeIsActive()) {
            // getting input from the gamepad
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            // setting motor powers based on the gamepad inputs
            motorFrontRight.setPower(vertical + horizontal - pivot);
            motorFrontLeft.setPower(vertical - horizontal + pivot);
            motorBackRight.setPower(vertical - horizontal - pivot);
            motorBackLeft.setPower(vertical + horizontal + pivot);

            if (gamepad1.a) {
                // intakes packing peanuts
                intakeMotor.setPower(INTAKE_POWER);
            }
            if (!gamepad1.a) {
                // stops intaking packing peanuts
                intakeMotor.setPower(0.0);
            }
            if (gamepad1.right_bumper) {
                // raises the slides
                slidesMotor.setPower(SLIDES_POWER);
                slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesMotor.setTargetPosition(TARGET_POS);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setTargetPosition(0);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}