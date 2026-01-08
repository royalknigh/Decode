package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

public class LaunchSystem {
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private DcMotorEx lm1, lm2;

    private ElapsedTime launchTimer = new ElapsedTime();
    private int launchStep = 0;
    private boolean isLaunching = false;
    private double currentTargetVelocity = 1800.0;
    private double activeIntervalMs = 1500.0;

    // --- VELOCITY CONSTANTS ---
    public static final double HIGH_VELOCITY = 1800.0;
    public static final double LOW_VELOCITY = 1300.0;
    public static final double IDLE_VELOCITY = 900;
    public static final double INIT_VELOCITY = 800;

    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;
        this.lm1 = MotorConfig.launchMotor1;
        this.lm2 = MotorConfig.launchMotor2;

        // Active velocity control on both motors
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setDualVelocity(INIT_VELOCITY);
    }

    private void setDualVelocity(double velocity) {
        lm1.setVelocity(velocity);
        lm2.setVelocity(velocity);
    }

    public void start(double target, double interval) {
        this.currentTargetVelocity = target;
        this.activeIntervalMs = interval;
        launchTimer.reset();
        launchStep = 0;
        isLaunching = true;
        setDualVelocity(currentTargetVelocity);
    }

    public void idle() {
        isLaunching = false;
        setDualVelocity(IDLE_VELOCITY);
    }

    public boolean update() {
        if (!isLaunching) return true;

        // --- EQUAL INTERVAL SEQUENCE ---

        // STEP 0: First ball prep
        if (launchStep == 0 && launchTimer.milliseconds() >= activeIntervalMs+100) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_MID);
            launchStep = 1;
            launchTimer.reset();
        }
        // STEP 1: Reset servo and feed next ball
        else if (launchStep == 1 && launchTimer.milliseconds() >= activeIntervalMs) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            MotorConfig.intakeMotor.setPower(1);
            launchStep = 2;
            launchTimer.reset();
        }
        // STEP 2: Stop intake and prep second ball
        else if (launchStep == 2 && launchTimer.milliseconds() >= activeIntervalMs) {
            MotorConfig.intakeMotor.setPower(0);
            servoConfig.launchServo.setPosition(ServoConstants.launch_MID);
            launchStep = 3;
            launchTimer.reset();
        }
        // STEP 3: Final PUSH
        else if (launchStep == 3 && launchTimer.milliseconds() >= activeIntervalMs) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
            launchStep = 4;
            launchTimer.reset();
        }
        // STEP 4: Cleanup/Idle
        else if (launchStep == 4 && launchTimer.milliseconds() >= 500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            idle();
            return true;
        }

        return false;
    }

    public String getStatus() {
        if (!isLaunching) return "IDLE @ 1000";
        double remaining = (activeIntervalMs - launchTimer.milliseconds()) / 1000.0;
        if (remaining < 0) remaining = 0;

        String mode = (activeIntervalMs <= 500) ? "RAPID" : "NORMAL";
        return String.format("%s: Step %d - %.1fs", mode, launchStep, remaining);
    }

    public double getVelocity() {
        return (lm1.getVelocity() + lm2.getVelocity()) / 2.0;
    }

    public void fullStop() {
        isLaunching = false;
        setDualVelocity(0);
    }
}