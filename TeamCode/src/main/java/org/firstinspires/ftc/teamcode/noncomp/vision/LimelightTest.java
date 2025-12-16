package org.firstinspires.ftc.teamcode.noncomp.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Limelight Turret Tracker (Rate Limited)", group = "Vision")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight = null;
    private CRServo turretServo = null;

    private static final double TARGET_X_OFFSET = 0.0;
    private static final double TURRET_KP = 0.01;
    private static final double ERROR_DEADBAND_DEGREES = 1.0;
    private static final double MAX_SERVO_SPEED = 0.5;

    private static final double CYCLE_TIME_SECONDS = 0.02;
    private static final long CYCLE_TIME_MILLIS = (long) (CYCLE_TIME_SECONDS * 1000);

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret_servo");
        turretServo.setPower(0.0);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);

        telemetry.addData("Status", "Limelight Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        long nextCycleStartTime = System.currentTimeMillis();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                long currentTime = System.currentTimeMillis();

                if (currentTime < nextCycleStartTime) {
                    long waitTime = nextCycleStartTime - currentTime;
                    if (waitTime > 0) {
                        sleep(waitTime);
                    }
                }

                nextCycleStartTime = System.currentTimeMillis() + CYCLE_TIME_MILLIS;

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    double tx = result.getTx();
                    double yawError = TARGET_X_OFFSET - tx;

                    if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {

                        double drivePower = yawError * TURRET_KP;

                        drivePower = Math.max(-MAX_SERVO_SPEED, Math.min(drivePower, MAX_SERVO_SPEED));

                        turretServo.setPower(drivePower);

                        telemetry.addData("Tracking", "Target Found!");
                        telemetry.addData("Yaw Error (tx)", "%.2f deg", yawError);

                    } else {
                        turretServo.setPower(0.0);
                        telemetry.addData("Tracking", "Centered and Stopped");
                    }
                } else {
                    turretServo.setPower(0.0);
                    telemetry.addData("Tracking", "No Valid Limelight Target");
                }

                telemetry.addData("Loop Frequency", "%.0f Hz", 1000.0 / (System.currentTimeMillis() - currentTime));
                telemetry.update();
            }
        }
    }
}