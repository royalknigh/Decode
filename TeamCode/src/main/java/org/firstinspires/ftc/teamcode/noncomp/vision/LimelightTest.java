package org.firstinspires.ftc.teamcode.noncomp.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.lang.Math;
import java.util.List;

@TeleOp(name = "Limelight Turret Tracker (Revised)", group = "Vision")
@Disabled
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight = null;
    private CRServo turretServo = null;



    private static final double TARGET_X_OFFSET = 0.0;
    private static final double TURRET_KP = 0.03;
    private static final double ERROR_DEADBAND_DEGREES = 1.0;
    private static final double MAX_SERVO_SPEED = 1;

    private boolean isTrackingEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret_servo");
        turretServo.setPower(0.0);

        // --- Limelight Configuration ---
        limelight.pipelineSwitch(5);
        // Explicitly start polling for data
        limelight.start();

        telemetry.addData("Status", "Limelight Initialized. Waiting for Start.");
        telemetry.addData("Controls", "Press A to TOGGLE Auto-Tracking");
        telemetry.update();

        waitForStart();

        // The single read before the loop is removed.
        // All continuous reading now happens inside the main loop.

        while (opModeIsActive()) {

            long currentTime = System.currentTimeMillis();

            // Note: gamepad1.aWasPressed() is a recommended method for a single press
            if (gamepad1.aWasPressed()) {
                isTrackingEnabled = !isTrackingEnabled;
            }

            LLResult result = limelight.getLatestResult();

            if (isTrackingEnabled) {

                if (result.isValid()) {

                    // --- AprilTag Tracking Data (Fiducial Results) ---
                    // CHECK FIDUCIAL RESULTS CONTINUOUSLY INSIDE THE LOOP
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    if (!fiducialResults.isEmpty()) {
                        // For turret control, we will likely only care about the *primary* target's tx
                        // but displaying the list is useful for debugging.
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                                    fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        }

                        // Use the primary target's tx (Limelight's default behavior)
                        double tx = result.getTx();

                        // Use the Botpose_MT2 for 3D Pose (assuming this is correct for your setup)
                        Pose3D botpose = result.getBotpose_MT2();
                        double zTranslation = botpose.getPosition().z; // Z is typically forward/backward
                        double yTranslation = botpose.getPosition().y; // Y is typically up/down or lateral (depending on Limelight mounting)

                        // Calculate Hypotenuse distance (assuming Z and Y define the right triangle legs)
                        double trueDistance = Math.hypot(zTranslation, yTranslation);

                        // PID-like calculation for yaw control
                        // Fixed Sign:
                        double yawError = tx - TARGET_X_OFFSET;

                        if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {

                            double drivePower = yawError * TURRET_KP;

                            // Clamp the power to the maximum speed
                            drivePower = Math.max(-MAX_SERVO_SPEED, Math.min(drivePower, MAX_SERVO_SPEED));

                            turretServo.setPower(drivePower);

                            telemetry.addData("Mode", "AUTO-TRACKING (ON)");
                            telemetry.addData("Tracking", "Target Found!");
                            telemetry.addData("Yaw Error (tx)", "%.2f deg", yawError);

                        } else {
                            turretServo.setPower(0.0);
                            telemetry.addData("Mode", "AUTO-TRACKING (ON)");
                            telemetry.addData("Tracking", "Centered and Stopped");
                        }

                        telemetry.addData("Distance (Hypot)", "%.2f meters", trueDistance);

                    } else {
                        // Valid result, but no fiducial tag found
                        turretServo.setPower(0.0);
                        telemetry.addData("Mode", "AUTO-TRACKING (ON)");
                        telemetry.addData("Tracking", "No AprilTag Found (Valid Result)");
                        telemetry.addData("Distance (Hypot)", "N/A");
                    }

                } else {
                    // Invalid result (no targets of any kind found)
                    turretServo.setPower(0.0);
                    telemetry.addData("Mode", "AUTO-TRACKING (ON)");
                    telemetry.addData("Tracking", "No Valid Limelight Target (Invalid Result)");
                    telemetry.addData("Distance (Hypot)", "N/A");
                }

            } else {

                // Manual Mode (Tracking Disabled)
                turretServo.setPower(0.0);

                // Still display the latest fiducial results for debugging in manual mode
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                                fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }

                telemetry.addData("Mode", "MANUAL (OFF)");
                telemetry.addData("Tracking", "Disabled");
                telemetry.addData("Distance (Hypot)", "N/A");
            }

            telemetry.addData("Loop Frequency", "%.0f Hz", 1000.0 / (System.currentTimeMillis() - currentTime));
            telemetry.update();
        }

        // Stop polling when OpMode ends
        limelight.stop();
    }
}