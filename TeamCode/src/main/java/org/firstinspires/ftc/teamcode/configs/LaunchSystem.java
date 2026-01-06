package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

public class LaunchSystem {
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private ElapsedTime launchTimer = new ElapsedTime();
    private int launchStep = 0;
    private boolean isLaunching = false;
    private int stabilityCounter = 0;

    // --- CONFIGURATION ---
    // Target is 1900 (approx 90% of your 2100 max) for better PID recovery
    private static final double TARGET_TICKS_PER_SEC = 2500;
    private static final double VELOCITY_TOLERANCE = 45.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double BELT_RATIO = 22.0 / 24.0;

    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;

        if (motorConfig.launchMotor1 instanceof DcMotorEx) {
            DcMotorEx m1 = (DcMotorEx) motorConfig.launchMotor1;
            DcMotorEx m2 = (DcMotorEx) motorConfig.launchMotor2;

            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void start() {
        launchTimer.reset();
        launchStep = 0;
        stabilityCounter = 0;
        isLaunching = true;
        setFlywheelVelocity(TARGET_TICKS_PER_SEC);
    }

    public boolean update() {
        if (!isLaunching) return true;

        double currentVel = getVelocity();

        // STEP 0: Wait for motor to stabilize
        if (launchStep == 0) {
            boolean atSpeed = Math.abs(TARGET_TICKS_PER_SEC - currentVel) < VELOCITY_TOLERANCE;

            if (atSpeed) {
                stabilityCounter++;
            } else {
                stabilityCounter = 0;
            }

            // Fire if stable for 3 frames or after 4s timeout
            if (stabilityCounter >= 3 || launchTimer.milliseconds() > 4000) {
                motorConfig.intakeMotor.setPower(0.8);
                servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
                launchStep = 1;
                launchTimer.reset();
            }
        }
        // STEP 1: Retract and Reset
        else if (launchStep == 1 && launchTimer.milliseconds() >= 500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            motorConfig.intakeMotor.setPower(0);
            stop(); // Turn off flywheel after shot
            return true;
        }
        return false;
    }

    public void stop() {
        isLaunching = false;
        launchStep = 0;
        stabilityCounter = 0;
        setFlywheelVelocity(0);
        motorConfig.intakeMotor.setPower(0);
    }

    private void setFlywheelVelocity(double velocity) {
        if (motorConfig.launchMotor1 instanceof DcMotorEx) {
            ((DcMotorEx) motorConfig.launchMotor1).setVelocity(velocity);
            ((DcMotorEx) motorConfig.launchMotor2).setVelocity(velocity);
        }
    }

    // --- GETTERS FOR TELEMETRY ---
    public double getVelocity() {
        return (motorConfig.launchMotor1 instanceof DcMotorEx) ?
                ((DcMotorEx) motorConfig.launchMotor1).getVelocity() : 0;
    }

    public double getFlywheelRPM() {
        // (TicksPerSec / 28) * 60 = Motor RPM. Then multiply by Belt Ratio.
        return (getVelocity() / TICKS_PER_REV) * 60.0 * BELT_RATIO;
    }

    public double getTargetRPM() {
        return (TARGET_TICKS_PER_SEC / TICKS_PER_REV) * 60.0 * BELT_RATIO;
    }

    public boolean isReady() {
        return stabilityCounter >= 3;
    }
}