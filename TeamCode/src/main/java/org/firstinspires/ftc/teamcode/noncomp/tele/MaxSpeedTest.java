package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Flywheel Max Speed Test", group="Calibration")
public class MaxSpeedTest extends LinearOpMode {

    private DcMotorEx launchMotor1;

    @Override
    public void runOpMode() {
        // Hardware Map
        launchMotor1 = hardwareMap.get(DcMotorEx.class, "lm1"); // Use your config name
        
        // Ensure we are in a mode that tracks ticks
        launchMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use RAW power for the test

        telemetry.addLine("Hold RIGHT BUMPER to run at 100% power.");
        telemetry.addLine("Wait for 'Actual Velocity' to level out.");
        telemetry.update();

        waitForStart();

        double maxObservedVelocity = 0;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                launchMotor1.setPower(1.0);
                
                // Track the highest speed reached
                double currentVel = Math.abs(launchMotor1.getVelocity());
                if (currentVel > maxObservedVelocity) {
                    maxObservedVelocity = currentVel;
                }
            } else {
                launchMotor1.setPower(0);
            }

            // Calculate the "Safe Zone" (85% of max)
            double safeTarget = maxObservedVelocity * 0.85;

            telemetry.addLine("=== CALIBRATION ===");
            telemetry.addData("Current Velocity", launchMotor1.getVelocity());
            telemetry.addData("MAX REACHED", maxObservedVelocity);
            telemetry.addLine("-------------------");
            telemetry.addData("RECOMMENDED TARGET", (int)safeTarget);
            telemetry.addLine("\nUse the Recommended Target in your LaunchSystem config.");
            telemetry.update();
        }
    }
}