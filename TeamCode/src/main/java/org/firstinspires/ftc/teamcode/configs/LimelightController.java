package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;

public class LimelightController {
    private Limelight3A limelight;
    private ServoConfig servoConfig;
    private boolean isTrackingEnabled = false;

    private static final double TARGET_X_OFFSET = 0.0;
    private static final double TURRET_KP = 0.01;
    private static final double ERROR_DEADBAND_DEGREES = 1.0;
    private static final double MAX_SERVO_SPEED = 1;

    private static final double MOUNT_ANGLE_DEG = 21.23;
    private static final double LENS_HEIGHT_INCHES = 15.0;
    private static final double GOAL_HEIGHT_INCHES = 28.75;

    public LimelightController(Limelight3A limelight, ServoConfig servoConfig) {
        this.limelight = limelight;
        this.servoConfig = servoConfig;
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    public void toggleTracking() {
        isTrackingEnabled = !isTrackingEnabled;
    }

    public void updateTracking() {
        LLResult result = limelight.getLatestResult();

        if (isTrackingEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            double yawError = tx - TARGET_X_OFFSET;

            if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {
                double turretPower = yawError * TURRET_KP;
                turretPower = Math.max(-MAX_SERVO_SPEED, Math.min(turretPower, MAX_SERVO_SPEED));
                servoConfig.leftTurretServo.setPower(turretPower);
                servoConfig.rightTurretServo.setPower(turretPower);
            } else {
                servoConfig.leftTurretServo.setPower(0.0);
                servoConfig.rightTurretServo.setPower(0.0);
            }
        } else {
            servoConfig.leftTurretServo.setPower(0.0);
            servoConfig.rightTurretServo.setPower(0.0);
        }
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEG + ty);
            double rawDist = (GOAL_HEIGHT_INCHES - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
            double polyDist = (0.0011 * Math.pow(rawDist, 2)) + (0.64 * rawDist) + 11.5;

            if (polyDist < 120.0) {
                return polyDist;
            } else {
                double overhead = polyDist - 120.0;
                return 120.0 + (overhead * 0.25);
            }
        }
        return -1; // No valid target
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}