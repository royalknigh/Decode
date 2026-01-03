package org.firstinspires.ftc.teamcode.noncomp.tele; // Assuming a package structure

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

@TeleOp(name = "Flywheel Tuner Tutorial", group = "Tuning")
@Disabled
public class FlywheelTest extends OpMode {

    // --- Motor and Velocity ---
    private DcMotorEx flywheelMotor;

    // Target Velocities (Constants based on video's logic)
    public double highVelocity = 1500.0;
    public double lowVelocity = 900.0;
    public double curTargetVelocity;

    // --- PIDF Constants ---
    public double P = 0.0;
    public double F = 0.0;

    // --- Step Size Control (from image_11d246.jpg) ---
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        // --- Hardware Initialization ---
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "motor");

        // --- Motor Configuration (from image_11d246.jpg) ---
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Initial PIDF Coeffs (from image_11d246.jpg) ---
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // --- Initial State ---
        curTargetVelocity = highVelocity;

        telemetry.addLine("Init Complete"); // From image_11d246.jpg
    }

    @Override
    public void loop() {
        // --- 1. Gamepad Commands (from image_11d208.png) ---

        // Toggle Velocity (Y button)
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        // Cycle Step Index (B button)
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F Value (D-Pad Left/Right)
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // Adjust P Value (D-Pad Up/Down)
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        // Note: The video image shows dpadUpWasPressed() here, which is an error.
        // It is corrected to dpadDownWasPressed() based on functional intent.
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // Ensure P and F don't go negative (Safety addition)
        P = Math.max(0.0, P);
        F = Math.max(0.0, F);

        // --- 2. Update Motor (from image_11cf42.jpg) ---

        // Set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set velocity
        flywheelMotor.setVelocity(curTargetVelocity);

        // --- 3. Telemetry Update (from image_11cf42.jpg) ---

        // Get current velocity and calculate error
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("----------");
        telemetry.addData("Tuning P (D-Pad U/D)", "%.4f", P);
        telemetry.addData("Tuning F (D-Pad L/R)", "%.4f", F);
        telemetry.addData("Step Size (B Button)", "%.4f", stepSizes[stepIndex]);
        telemetry.update();
    }
}