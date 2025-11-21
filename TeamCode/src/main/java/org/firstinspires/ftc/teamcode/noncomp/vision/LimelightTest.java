package org.firstinspires.ftc.teamcode.noncomp.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Limelight AprilTag Distance Only")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(10);
        limelight.start();

        telemetry.addData("Status", "Initialized - Waiting for START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

                if (!aprilTags.isEmpty()) {

                    LLResultTypes.FiducialResult tag = aprilTags.get(0);

                    int tagId = tag.getFiducialId();
                    Pose3D pose = tag.getRobotPoseTargetSpace();

                    double distanceInches = Math.hypot(
                            Math.hypot(pose.getPosition().x, pose.getPosition().y),
                            pose.getPosition().z
                    );
                    double distanceForwardInches = pose.getPosition().z;

                    telemetry.addData("Detected Tag ID", tagId);
                    telemetry.addData("Distance (3D Total)", "%.2f inches", distanceInches);
                    telemetry.addData("Distance Forward (Z)", "%.2f inches", distanceForwardInches);
                    telemetry.addData("X Offset", "%.2f in", pose.getPosition().x);
                    telemetry.addData("Y Offset", "%.2f in", pose.getPosition().y);
                } else {
                    telemetry.addData("AprilTags", "None detected");
                }
            } else {
                telemetry.addLine("No valid Limelight result");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}