package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;
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
    private double currentTargetVelocity = 0;

    // --- PANELS CONFIGURABLE CONSTANTS ---

    public static double P = 50;

    public static double F = 14.2;
    public static double highVelocity = 1700.0; // Must be public static

    public static double lowVelocity = 1300.0;

    public static double activeIntervalMs = 600;
    public static double prepDelayMs = 200.0;

    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;
        this.lm1 = MotorConfig.launchMotor1;
        this.lm2 = MotorConfig.launchMotor2;

        // Use EX methods for better velocity control
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        updatePIDF(); // Apply initial P and F
    }

    /**
     * Call this at the start of loop() or whenever you change values in Panels
     */
    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }

    private void setDualVelocity(double velocity) {
        lm1.setVelocity(velocity);
        lm2.setVelocity(velocity);
    }

    public void start(double target, double activeIntervalMs) {
        this.currentTargetVelocity = target;
        launchTimer.reset();
        launchStep = 0;
        isLaunching = true;
        setDualVelocity(currentTargetVelocity);
    }

    public void idle() {
        isLaunching = false;
        setDualVelocity(900.0); // IDLE_VELOCITY
    }

    public boolean update() {
        if (!isLaunching) return true;

        // Ensure PIDF values from the Panel are applied during the run
        updatePIDF();

        // STEP 0: First ball prep
        if (launchStep == 0 && launchTimer.milliseconds() >= (activeIntervalMs + prepDelayMs)) {
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
        else if (launchStep == 2 && launchTimer.milliseconds() >= (activeIntervalMs + prepDelayMs)) {
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
    public void fullStop() {
        isLaunching = false;    // Kill the state machine
        launchStep = 0;         // Reset the sequence
        setDualVelocity(0);      // Stop flywheel motors

        // Safety: Reset the launch servo to its start position
        servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);

        // Stop any auxiliary motors like the intake

}}