package org.firstinspires.ftc.teamcode.noncomp.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Sensor: Limelight Distance", group = "Sensor")
public class DistanceTest extends LinearOpMode {

    private Limelight3A limelight;

    /* * CONSTANTS FOR DISTANCE CALCULATION
     * Adjust these based on your physical robot setup
     */
    final double MOUNT_ANGLE_DEG = 28.2;     // Degrees back from vertical
    final double LENS_HEIGHT_INCHES = 6;  // Center of lens to floor
    final double GOAL_HEIGHT_INCHES = 28.75;  // Center of AprilTag to floor

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        // Ensure your pipeline (index 5 here) is set to AprilTags in the LL web UI
        limelight.pipelineSwitch(5);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // 1. GET VERTICAL OFFSET (ty)
                double ty = result.getTy();

                // 2. CALCULATE DISTANCE USING YOUR FORMULA
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEG + ty);
                double distanceInches = (GOAL_HEIGHT_INCHES - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // 3. TELEMETRY OUTPUT
                telemetry.addData("Target", "FOUND");
                telemetry.addData("Vertical Offset (ty)", "%.2f degrees", ty);
                telemetry.addData("Calculated Distance", "%.2f inches", distanceInches);

                // Show individual AprilTag IDs if multiple are in view
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial ID", fr.getFiducialId());
                }

            } else {
                telemetry.addData("Target", "NOT VISIBLE");
            }

            telemetry.addData("LL Temp", "%.1fC", status.getTemp());
            telemetry.update();
        }
        limelight.stop();
    }
}