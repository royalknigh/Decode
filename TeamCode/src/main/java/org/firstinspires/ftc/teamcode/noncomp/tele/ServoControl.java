package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Simple Servo")
public class ServoControl extends LinearOpMode {

    private Servo servo;
    private double position =0;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            if(-gamepad1.left_stick_y > 0)
                position +=0.002;
            if(-gamepad1.left_stick_y<0)
                position -= 0.002;

            servo.setPosition(position);
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
