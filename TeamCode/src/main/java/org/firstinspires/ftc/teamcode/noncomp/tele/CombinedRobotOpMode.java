package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name="Robot: Drive + Turret + Launch", group="Final")
public class CombinedRobotOpMode extends LinearOpMode {

    // --- HARDWARE ---
    private ElapsedTime runtime = new ElapsedTime();
    
    // Drivetrain
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor launchMotor1 = null;
    private DcMotor launchMotor2 = null;
    private DcMotor intakeMotor = null;


    private CRServo turretServo = null;
    private Servo hoodServo = null;
    private Servo launchServo = null;
    private Limelight3A limelight = null;

    // --- TUNING ---
    private static final double TARGET_X_OFFSET = 0.0;
    private static final double TURRET_KP = 0.02;
    private static final double ERROR_DEADBAND_DEGREES = 1.0;
    private static final double MAX_SERVO_SPEED = 0.7;

    private double hoodPosition =0.5;
    private double servoDown = 0.51;
    private double servoUp = 0.9;

    private boolean isTrackingEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. Drivetrain Initialization
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Turret & Launch Motor Initialization
        turretServo = hardwareMap.get(CRServo.class, "leftTurret");
        launchMotor1 = hardwareMap.get(DcMotor.class, "lm1");
        launchMotor2 = hardwareMap.get(DcMotor.class, "lm2");
        intakeMotor = hardwareMap.get(DcMotor.class, "im");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        launchServo = hardwareMap.get(Servo.class, "ls");

        launchMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launchServo.setDirection(Servo.Direction.REVERSE);

        // 3. Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5); 
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =========================================================
            // A. DRIVETRAIN (Mecanum / Omni)
            // =========================================================
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double flP = axial + lateral + yaw;
            double frP = axial - lateral - yaw;
            double blP = axial - lateral + yaw;
            double brP = axial + lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.abs(flP), Math.abs(frP));
            max = Math.max(max, Math.abs(blP));
            max = Math.max(max, Math.abs(brP));
            if (max > 1.0) {
                flP /= max; frP /= max; blP /= max; brP /= max;
            }

            frontLeftDrive.setPower(flP);
            frontRightDrive.setPower(frP);
            backLeftDrive.setPower(blP);
            backRightDrive.setPower(brP);


            // =========================================================
            // B. LAUNCHER / INTAKE CONTROL
            // =========================================================
            // Combined logic from the second script: Right trigger (pos), Left trigger (neg)
            double launchPower = gamepad1.left_trigger;
            launchMotor1.setPower(launchPower);
            launchMotor2.setPower(launchPower);

            double intakePower = gamepad1.right_trigger;
            intakeMotor.setPower(intakePower);

            if(gamepad1.dpad_up)
                hoodPosition+=0.03;
            if(gamepad1.dpad_down)
                hoodPosition-=0.03;

            if(gamepad1.b)
                launchServo.setPosition(servoUp);
            else
                launchServo.setPosition(servoDown);
            hoodServo.setPosition(hoodPosition);



            // =========================================================
            // C. TURRET TRACKING (Limelight)
            // =========================================================
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
                    turretServo.setPower(turretPower);
                } else {
                    turretServo.setPower(0.0);
                }
            } else {
                turretServo.setPower(0.0); // Stop turret if manual or no target
            }

            // =========================================================
            // D. TELEMETRY
            // =========================================================
            telemetry.addData("Tracking", isTrackingEnabled ? "AUTO" : "MANUAL");
            telemetry.addData("Launch Power", "%.2f", launchPower);
            telemetry.addData("Turret Power", "%.2f", turretServo.getPower());

            telemetry.update();
        }

        limelight.stop();
    }
}