/*
 * Copyright (c) 2024 Limelight Vision & FIRST. All rights reserved.
 *
 * Acest cod combină un driver de bază Omni/Mecanum cu urmărirea turelei AprilTag Limelight.
 */

package org.firstinspires.ftc.teamcode.noncomp.vision; // Asigurați-vă că pachetul este corect

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;
import java.lang.Math;


@TeleOp(name="Apriltag follower", group="Final OpModes")
public class SensorLimelight3A extends LinearOpMode {

    // --- VARIABILE HARDWARE ---
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private CRServo turretServo = null; // Turela (Servo cu Rotație Continuă)
    private Limelight3A limelight = null;

    // --- VARIABILE DE REGLARE (TUNING) ---
    private static final double TARGET_X_OFFSET = 0.0; // Centrare la 0 grade
    private static final double TURRET_KP = 0.02; // Coeficient Proporțional (începeți cu 0.01 sau 0.005 dacă oscilează)
    private static final double ERROR_DEADBAND_DEGREES = 1.0; // Toleranță de eroare
    private static final double MAX_SERVO_SPEED = 0.5; // Viteza maximă a turelei

    // --- STAREA ROBOTULUI ---
    private boolean isTrackingEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- I. INIȚIALIZARE HARDWARE ---

        // 1. Inițializare motoare de acționare
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


        // 2. Inițializare Turelă și Limelight
        turretServo = hardwareMap.get(CRServo.class, "turret_servo");
        turretServo.setPower(0.0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5); // Setați pipeline-ul AprilTag (Ajustați dacă este necesar)
        limelight.start(); // Începe colectarea datelor

        telemetry.setMsTransmissionInterval(50);


        // --- II. AȘTEPTARE START ---

        telemetry.addData("Status", "Inițializare finalizată.");
        telemetry.addData("Controale", "Stick Stanga: Drive | Stick Dreapta: Rotire | A: TOGGLE Tracking Turelă");
        telemetry.addData("Tracking", "Mod: MANUAL (OFF)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // --- III. BUCLE DE FUNCȚIONARE ---

        while (opModeIsActive()) {

            // =========================================================
            // A. CONTROLUL BAZA DE RULARE (DRIVETRAIN)
            // =========================================================

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x; // Yaw pentru control manual

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalizarea puterilor
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Aplicarea puterii la motoare
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            // =========================================================
            // B. CONTROLUL TURELEI ȘI VIZIUNEA
            // =========================================================

            // Comutare stare urmărire
            if (gamepad1.aWasPressed()) {
                isTrackingEnabled = !isTrackingEnabled;
            }

            LLResult result = limelight.getLatestResult();

            if (isTrackingEnabled) {

                if (result.isValid()) {

                    // Verifică dacă s-au găsit AprilTag-uri
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                    if (!fiducialResults.isEmpty()) {

                        double tx = result.getTx(); // Eroarea de unghi orizontal a țintei principale

                        // FIXUL DE SEMN: Calculul erorii
                        // Pentru ca turela să urmărească, eroarea ar trebui să fie:
                        // tx > 0 (ținta la dreapta) -> power > 0 (rotiți la stânga)
                        // tx < 0 (ținta la stânga) -> power < 0 (rotiți la dreapta)
                        // Dacă servo-ul dvs. face invers, schimbați semnul $K_P$.
                        double yawError = tx - TARGET_X_OFFSET; // Eroarea este pur și simplu 'tx' dacă TARGET_X_OFFSET = 0

                        if (Math.abs(yawError) > ERROR_DEADBAND_DEGREES) {

                            double drivePower = yawError * TURRET_KP;

                            // Limitează puterea
                            drivePower = Math.max(-MAX_SERVO_SPEED, Math.min(drivePower, MAX_SERVO_SPEED));

                            turretServo.setPower(drivePower);

                            telemetry.addData("Tracking Status", "AUTO-TRACKING (ON)");
                            telemetry.addData("Yaw Error (tx)", "%.2f deg", yawError);

                        } else {
                            // În deadband
                            turretServo.setPower(0.0);
                            telemetry.addData("Tracking Status", "Centered and Stopped");
                        }

                        // Afișează distanța estimată și ID-ul tag-ului
                        Pose3D botpose = result.getBotpose_MT2();
                        double zTranslation = botpose.getPosition().z; // Z este de obicei înainte/înapoi
                        double yTranslation = botpose.getPosition().y; // Y este de obicei în sus/jos sau lateral
                        double trueDistance = Math.hypot(zTranslation, yTranslation);

                        telemetry.addData("Tag Found", "ID: %d", fiducialResults.get(0).getFiducialId());
                        telemetry.addData("Distance (Hypot)", "%.2f meters", trueDistance);

                    } else {
                        // Limelight Valid, dar nu s-a găsit AprilTag
                        turretServo.setPower(0.0);
                        telemetry.addData("Tracking Status", "No AprilTag Found (Valid Result)");
                    }

                } else {
                    // Rezultat Invalid (nu s-a găsit nimic)
                    turretServo.setPower(0.0);
                    telemetry.addData("Tracking Status", "No Valid Limelight Target");
                }

            } else {
                // Mod Manual
                turretServo.setPower(0.0);
                telemetry.addData("Tracking Status", "MANUAL (OFF)");

                // Afișează tx dacă este vizibil (pentru depanare)
                if(result.isValid()) {
                    telemetry.addData("Limelight tx", "%.2f deg", result.getTx());
                }
            }


            // =========================================================
            // C. AFIȘARE TELEMETRIE
            // =========================================================

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Turret Power", turretServo.getPower());
            telemetry.addData("Drivetrain Power (FL/FR)", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.update();
        }

        // --- IV. OPRIRE ---
        limelight.stop();
        turretServo.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }
}
