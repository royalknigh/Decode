/* Copyright (c) 2017 FIRST. All rights reserved.
 * [License details omitted for brevity, same as provided]
 */
package org.firstinspires.ftc.teamcode.noncomp.tele;

import static org.firstinspires.ftc.teamcode.configs.MotorConfig.launchMotor1;
import static org.firstinspires.ftc.teamcode.configs.MotorConfig.launchMotor2;

import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

import java.util.List;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class Tele extends OpMode {
    private ElapsedTime runtime = new ElapsedTime(), launchTimer = new ElapsedTime();

    private enum State {INIT, INTAKE, LAUNCH, LIFT}

    private State state;
    private final int fraction = 1;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private boolean empty = false;
    private boolean isTrackingEnabled = false;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Limelight3A limelight = null;

    // --- VARIABILE DE REGLARE (TUNING) ---
    private static final double TARGET_X_OFFSET = 0.0; // Centrare la 0 grade
    private static final double TURRET_KP = 0.018; // Coeficient Proporțional (începeți cu 0.01 sau 0.005 dacă oscilează)
    private static final double ERROR_DEADBAND_DEGREES = 1.0; // Toleranță de eroare
    private static final double MAX_SERVO_SPEED = 0.8; // Viteza maximă a turelei


    private int launchCount = 0;
    private int launchStep = 0;
    boolean resetTimer = true;


    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servoConfig.setInitPos();
        state = State.INIT;
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        movement();
        robotTracking();
//      motorConfig.setLiftPID();
//      motorConfig.updatePIDFController();
        stateMachine();
        telemetry.addData("state :", state);
    }

    @Override
    public void stop() {
    }

    public void movement() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        motorConfig.setMotorPowers(frontLeftPower / fraction, backLeftPower / fraction,
                frontRightPower / fraction, backRightPower / fraction);

    }
    public void robotTracking(){
        if (gamepad1.aWasPressed()) { // Toggle tracking with A button
            isTrackingEnabled = !isTrackingEnabled;
        }

        LLResult result = limelight.getLatestResult();

        if (isTrackingEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            double yawError = tx - TARGET_X_OFFSET;

            if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {
                double turretPower = yawError * TURRET_KP;
                turretPower = Math.max(-MAX_SERVO_SPEED, Math.min(turretPower, MAX_SERVO_SPEED));
                servoConfig.leftTurretServo.setPower(turretPower);
            } else {
                servoConfig.leftTurretServo.setPower(0.0);
            }
        } else {
            servoConfig.leftTurretServo.setPower(0.0); // Stop turret if manual or no target
        }
    }

    public void stateMachine() {
        switch (state) {
            case INIT: {
                motorConfig.launchMotor1.setPower(0);
                motorConfig.launchMotor2.setPower(0);
                motorConfig.intakeMotor.setPower(0);
                if (gamepad1.left_trigger > 0)
                    state = State.INTAKE;
                if (gamepad1.x) {
                    state = State.LAUNCH;
                    resetLaunch();
                }
                break;
            }
            case INTAKE: {
                if (gamepad1.left_trigger > 0.1)
                    motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
                else
                    motorConfig.intakeMotor.setPower(0);
                if (gamepad1.x) {
                    state = State.LAUNCH;
                    resetLaunch();
                }
                break;
            }
            case LAUNCH: {
                launch();

                break;
            }
            case LIFT: {
                // TODO: IMPLEMENT LIFT
                break;
            }
        }
    }
    public void launch(){
        if(resetTimer){
            resetTimer = false;
            launchMotor1.setPower(1);
            launchMotor2.setPower(1);
            launchTimer.reset();
        }

        if (launchStep == 0 && launchTimer.milliseconds() >= 700) {
            motorConfig.intakeMotor.setPower(0.8);
//            launchServo.setPosition(ServoConstants.launch_INIT);
            launchStep = 1;
            launchTimer.reset();
        } else if (launchStep == 1 && launchTimer.milliseconds() >= 2500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
            launchStep=2;
            launchTimer.reset();
//            motorConfig.intakeMotor.setPower(0);

        }if (launchStep >= 2 && launchTimer.milliseconds()>300) {
            resetTimer = true;
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            state = State.INIT;
        }
    }
    public void resetLaunch() {
        launchTimer.reset();
        launchCount = 0;
        launchStep = 0;
        resetTimer = true;
    }
}