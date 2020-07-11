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
    final long TIME_TO_EXTEND = 500;
    final double INITIAL_DUMPER_POSITION = 0.0;
    final double DUMPED_POSITION = 1.0;

    // hardware declarations
    private DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
    private DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
    private DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
    private DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

    private DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

    private DcMotor slidesMotor = hardwareMap.dcMotor.get("slidesMotor");
    private Servo dumperServo = hardwareMap.servo.get("dumperServo");

    @Override
    public void runOpMode() {
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
                // starts intaking packing peanuts
                intakeMotor.setPower(INTAKE_POWER);
            }
            if (gamepad1.b) {
                // stops intaking packing peanuts
                intakeMotor.setPower(0.0);
            }
            if (gamepad1.right_bumper) {
                // raises the slides
                slidesMotor.setPower(SLIDES_POWER);
                sleep(TIME_TO_EXTEND);
                slidesMotor.setPower(0.0);
                // dumps the balls
                dumperServo.setPosition(DUMPED_POSITION);
            }
        }
    }
}