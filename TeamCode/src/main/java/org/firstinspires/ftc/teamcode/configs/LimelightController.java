package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightController {
    private Limelight3A limelight;
    private ServoConfig servoConfig;
    private boolean isTrackingEnabled = false;

    // Adjusted constants for better physical response
    private static final double TARGET_X_OFFSET = 0.0;
    private static final double TURRET_KP = 0.02; // Increased from 0.01 for more "kick"
    private static final double ERROR_DEADBAND_DEGREES = 0.5;
    private static final double MAX_SERVO_SPEED = 1.0;

    private static final double MOUNT_ANGLE_DEG = 21.23;
    private static final double LENS_HEIGHT_INCHES = 15.0;
    private static final double GOAL_HEIGHT_INCHES = 28.75;

    public enum Alliance { BLUE, RED }

    public LimelightController(Limelight3A limelight, ServoConfig servoConfig) {
        this.limelight = limelight;
        this.servoConfig = servoConfig;
        limelight.pipelineSwitch(5); // Default Blue
        limelight.start();
    }

    public void setAlliance(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            limelight.pipelineSwitch(5);
        } else {
            limelight.pipelineSwitch(6);
        }
    }

    public void toggleTracking() {
        isTrackingEnabled = !isTrackingEnabled;
    }

    public boolean isTrackingEnabled() {
        return isTrackingEnabled;
    }

    public void updateTracking() {
        LLResult result = limelight.getLatestResult();

        // Must be enabled AND see a target to move
        if (isTrackingEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            double yawError = tx - TARGET_X_OFFSET;

            if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {
                double turretPower = yawError * TURRET_KP;
                turretPower = Math.max(-MAX_SERVO_SPEED, Math.min(turretPower, MAX_SERVO_SPEED));

                // Sending power to CRServos
                servoConfig.leftTurretServo.setPower(turretPower);
                servoConfig.rightTurretServo.setPower(turretPower);
            } else {
                stopTurret();
            }
        } else {
            stopTurret();
        }
    }

    private void stopTurret() {
        servoConfig.leftTurretServo.setPower(0.0);
        servoConfig.rightTurretServo.setPower(0.0);
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEG + ty);
            double rawDist = (GOAL_HEIGHT_INCHES - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
            return (0.0011 * Math.pow(rawDist, 2)) + (0.64 * rawDist) + 11.5;
        }
        return -1;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}