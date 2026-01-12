package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.configs.LimelightController;

@Configurable
@TeleOp(name="Full Control Competition TeleOp", group="Iterative OpMode")
public class Tele extends OpMode {

    // --- Configuration Variables (Visible in Panels) ---
    public static LimelightController.Alliance alliance = LimelightController.Alliance.BLUE;

    // --- Logic & State ---
    private enum State {INIT, INTAKE, LAUNCH}
    private State state;
    private boolean lastB = false; // Alliance Toggle
    private boolean lastA = false; // Turret Toggle
    private double hoodPosition=0.5;

    // --- Hardware Controllers ---
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;
    private LimelightController limelightController;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        launchSystem = new LaunchSystem(motorConfig, servoConfig);

        limelightController = new LimelightController(
                hardwareMap.get(Limelight3A.class, "limelight"),
                servoConfig
        );

        servoConfig.setInitPos();
        state = State.INIT;

        telemetry.addData("Status", "Initialized. B to Toggle Alliance.");
    }

    @Override
    public void init_loop() {
        // Alliance Toggle (B)
        if (gamepad1.b && !lastB) {
            alliance = (alliance == LimelightController.Alliance.BLUE) ?
                    LimelightController.Alliance.RED : LimelightController.Alliance.BLUE;
            limelightController.setAlliance(alliance);
            gamepad1.rumble(250);
        }
        lastB = gamepad1.b;

        telemetry.addLine("--- PRE-MATCH CONFIG ---");
        telemetry.addData("Alliance (B)", alliance);
        telemetry.addData("Pipeline", (alliance == LimelightController.Alliance.BLUE) ? "5 (Blue)" : "6 (Red)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- TURRET TOGGLE (A) ---
        if (gamepad1.a && !lastA) {
            limelightController.toggleTracking();
            if (limelightController.isTrackingEnabled()) {
                gamepad1.rumbleBlips(2); // Double pulse for ON
            } else {
                gamepad1.rumble(500);      // Long pulse for OFF
            }
        }
        lastA = gamepad1.a;

        // 1. Update Subsystems
        limelightController.updateTracking();

        // 2. Handle Driver Inputs
        handleMovement();
        handleStateMachine();
        handleHood();

        if(gamepad1.rightBumperWasPressed())
            launchSystem.fullStop();

        // 3. Combined Telemetry (Original + New)
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret Tracking (A)", limelightController.isTrackingEnabled() ? "ACTIVE" : "OFF");

        telemetry.addLine("\n=== LAUNCHER & HOOD ===");
        telemetry.addData("Launch Mode", launchSystem.getStatus());
        telemetry.addData("Flywheel Velocity", "%.0f", launchSystem.getVelocity());
        telemetry.addData("Hood Pos", "%.3f", hoodPosition);

        telemetry.addLine("\n=== TARGETING INFO ===");
        LLResult res = limelightController.getLatestResult();
        if (res != null && res.isValid()) {
            telemetry.addData("Limelight", "LOCKED");
            telemetry.addData("Distance", "%.2f in", limelightController.getDistance());
            telemetry.addData("Target X (TX)", "%.2f", res.getTx());
        } else {
            telemetry.addData("Limelight", "SEARCHING...");
        }

        telemetry.update();
    }

    private void handleMovement() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        motorConfig.setMotorPowers(
                (y + x + rx) / denominator, (y - x + rx) / denominator,
                (y - x - rx) / denominator, (y + x - rx) / denominator
        );
    }

    private void handleStateMachine() {
        switch (state) {
            case INIT:
                if (gamepad1.left_trigger > 0.1) state = State.INTAKE;
                if (gamepad1.x) {
                    launchSystem.start(LaunchSystem.HIGH_VELOCITY, 800);
                    state = State.LAUNCH;
                }
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.LOW_VELOCITY, 600);
                    state = State.LAUNCH;
                }
                break;

            case INTAKE:
                motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
                if (gamepad1.x) {
                    launchSystem.start(LaunchSystem.HIGH_VELOCITY, 800);
                    state = State.LAUNCH;
                }
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.LOW_VELOCITY, 600);
                    state = State.LAUNCH;
                }
                if (gamepad1.left_trigger <= 0.1) state = State.INIT;
                break;

            case LAUNCH:
                if (launchSystem.update()) {
                    state = State.INIT;
                }
                break;
        }
    }

    private void handleHood() {
       if (limelightController.getDistance() < 90) {
           double x = limelightController.getDistance();
           hoodPosition = 0.000235*x*x -0.03207*x+1.7471;
         hoodPosition = Range.clip(hoodPosition, 0, 0.98);

       }
       else
           hoodPosition = 0.98;

//        if(gamepad1.dpad_up)    hoodPosition+= 0.002;
//        if(gamepad1.dpad_down)    hoodPosition-= 0.002;
////
        servoConfig.hoodServo.setPosition(hoodPosition);
    }

}