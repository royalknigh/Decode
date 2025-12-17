package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CRServo Simple Control")
public class ServoTest extends LinearOpMode {

    private CRServo crServo1;
    private CRServo crServo2;

    private DcMotor motor;
    private DcMotor motor1;

    @Override
    public void runOpMode() {

        crServo1 = hardwareMap.get(CRServo.class, "crServo1");
        crServo2 = hardwareMap.get(CRServo.class, "crServo2");
        motor = hardwareMap.get(DcMotor.class, "fl");
        motor1 = hardwareMap.get(DcMotor.class, "bl");
        motor1 = hardwareMap.get(DcMotor.class, "bl");

        waitForStart();

        while (opModeIsActive()) {

            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            // value will be between -1 and +1

            crServo1.setPower(power);
            crServo2.setPower(power);
            motor.setPower(gamepad1.left_stick_y);
            motor1.setPower(gamepad1.right_stick_y);

            telemetry.addData("CRServo Power", power);
            telemetry.update();
        }
    }
}
