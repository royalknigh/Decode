package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

public class LaunchSystem {
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private DcMotorEx master, slave;

    private ElapsedTime launchTimer = new ElapsedTime();
    private int launchStep = 0;
    private boolean isLaunching = false;
    private int stabilityCounter = 0;

    // Hardcoded PIDF
    public static double P = 15.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.5;

    // Velocities
    private static final double TARGET_TICKS_PER_SEC = 2500;
    private static final double IDLE_TICKS_PER_SEC = 1000; // Fast enough to stay warm, slow enough to save battery
    private static final double VELOCITY_TOLERANCE = 45.0;

    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;

        master = (DcMotorEx) motorConfig.launchMotor1;
        slave = (DcMotorEx) motorConfig.launchMotor2;

        master.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        master.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        slave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        master.setDirection(DcMotorSimple.Direction.REVERSE);
        slave.setDirection(DcMotorSimple.Direction.FORWARD);

        master.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start the match in Idle
        idle();
    }

    public void start() {
        launchTimer.reset();
        launchStep = 0;
        stabilityCounter = 0;
        isLaunching = true;
        master.setVelocity(TARGET_TICKS_PER_SEC);
    }

    public void idle() {
        isLaunching = false;
        master.setVelocity(IDLE_TICKS_PER_SEC);
    }

    public void fullStop() {
        isLaunching = false;
        master.setVelocity(0);
        slave.setPower(0);
    }

    public boolean update() {
        // ALWAYS keep slave in sync with master's power draw
        slave.setPower(master.getPower());

        if (!isLaunching) return true;

        double currentVel = master.getVelocity();

        // STEP 0: Wait for stabilization
        if (launchStep == 0) {
            boolean atSpeed = Math.abs(TARGET_TICKS_PER_SEC - currentVel) < VELOCITY_TOLERANCE;
            if (atSpeed) stabilityCounter++;
            else stabilityCounter = 0;

            if (stabilityCounter >= 3 || launchTimer.milliseconds() > 4000) {
                motorConfig.intakeMotor.setPower(0.8);
                servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
                launchStep = 1;
                launchTimer.reset();
            }
        }
        // STEP 1: Reset and return to IDLE
        else if (launchStep == 1 && launchTimer.milliseconds() >= 500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            motorConfig.intakeMotor.setPower(0);
            idle(); // Instead of full stop, go back to idle speed
            return true;
        }
        return false;
    }

    public double getVelocity() { return master.getVelocity(); }
    public boolean isReady() { return stabilityCounter >= 3; }
}