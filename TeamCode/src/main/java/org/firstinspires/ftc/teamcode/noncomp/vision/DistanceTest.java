package org.firstinspires.ftc.teamcode.noncomp.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;

@TeleOp(name = "Limelight Distance Calibrated", group = "Sensor")
public class DistanceTest extends LinearOpMode {

    private Limelight3A limelight;

    // PHYSICAL CONSTANTS
    final double MOUNT_ANGLE_DEG = 21.23;
    final double LENS_HEIGHT_INCHES = 15.0;
    final double GOAL_HEIGHT_INCHES = 28.75;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(5);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            // ... inside your while(opModeIsActive()) loop ...

            if (result != null && result.isValid()) {
                double ty = result.getTy();

                // 1. CALCULATE RAW TRIG DISTANCE
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEG + ty);
                double rawDist = (GOAL_HEIGHT_INCHES - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // 2. APPLY POLYNOMIAL CORRECTION (Your current curve)
                double polyDist = (0.0011 * Math.pow(rawDist, 2)) + (0.64 * rawDist) + 11.5;

                // 3. APPLY DAMPENING (Only for 120+ inches)
                double finalDistance;
                if (polyDist < 120.0) {
                    finalDistance = polyDist;
                } else {
                    // We take everything above 120 and dampen it significantly
                    // If polyDist is 140, this becomes: 120 + (20 * 0.25) = 125
                    double overhead = polyDist - 120.0;
                    finalDistance = 120.0 + (overhead * 0.25);
                }

                // 4. TELEMETRY
                telemetry.addData("Vertical Offset (ty)", "%.2f", ty);
                telemetry.addData("Corrected Distance", "%.1f in", finalDistance);
            } else {
                telemetry.addData("Target", "NOT VISIBLE");
            }

            telemetry.addData("LL Temp", "%.1fC", status.getTemp());
            telemetry.update();
        }
        limelight.stop();
    }
}