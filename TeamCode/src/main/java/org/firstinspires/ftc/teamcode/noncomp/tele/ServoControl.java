package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Simple Servo")
public class ServoControl extends LinearOpMode {

    private Servo servo;
    private double position =0.51;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "ls");
        servo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if(-gamepad1.left_stick_y > 0)
                position+=0.001;
            if(-gamepad1.left_stick_y<0)
                position-=0.001;

            position = Range.clip(position, 0, 1);

            servo.setPosition(position);
            telemetry.addData("servo: ", position);

            telemetry.update();
        }
    }
}