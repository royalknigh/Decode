package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.noncomp.tele.Tele.F;
import static org.firstinspires.ftc.teamcode.noncomp.tele.Tele.P;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.noncomp.tele.Tele;

public class LaunchSystem {
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private DcMotorEx lm1, lm2;

    private ElapsedTime launchTimer = new ElapsedTime();
    private int launchStep = 0;
    private boolean isLaunching = false;
    private double currentTargetVelocity = 0;

    // --- CONSTANTS --- 1100 - 1500

    public static double highVelocity = 1600.0;
    public static double lowVelocity = 1250.0;
    public static double idleVelocity = 900;
    public static double activeIntervalMs = 600.0;
    public static double prepDelayMs = 350.0;  //250.0

    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;
        this.lm1 = MotorConfig.launchMotor1;
        this.lm2 = MotorConfig.launchMotor2;

        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        updatePIDF();
    }

    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }

    public void setDualVelocity(double velocity) {
        lm1.setVelocity(velocity);
        lm2.setVelocity(velocity);
    }

    public void start(double target, double interval) {
        this.currentTargetVelocity = target;
        // Fix: Update the class variable with the interval passed from TeleOp
        LaunchSystem.activeIntervalMs = interval;

        launchTimer.reset();
        launchStep = 0;
        isLaunching = true;
        setDualVelocity(currentTargetVelocity);
    }

    public void idle() {
        isLaunching = false;
        setDualVelocity(idleVelocity);
    }

    public boolean launchIntake(){
        if (!isLaunching) return true;

        updatePIDF();

        motorConfig.intakeMotor.setPower(1);
        if(launchTimer.milliseconds() >= 5000){
            motorConfig.intakeMotor.setPower(0);
            idle();
            return true;
        }
        return false;
    }

    public boolean update() {
        if (!isLaunching) return true;

        updatePIDF();

        // STEP 0: First ball prep
        if (launchStep == 0 && launchTimer.milliseconds() >= (activeIntervalMs + prepDelayMs)) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_MID);
            launchStep = 1;
            launchTimer.reset();
        }
        // STEP 1: Feed ball
        else if (launchStep == 1 && launchTimer.milliseconds() >= activeIntervalMs) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            MotorConfig.intakeMotor.setPower(1);
            launchStep = 2;
            launchTimer.reset();
        }
        // STEP 2: Prep second ball
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
        // STEP 4: Cleanup
        else if (launchStep == 4 && launchTimer.milliseconds() >= 500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            idle();
            return true;
        }
        return false;
    }

    public void fullStop() {
        isLaunching = false;
        launchStep = 0;
        setDualVelocity(0);
        servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
        MotorConfig.intakeMotor.setPower(0);
    }
}