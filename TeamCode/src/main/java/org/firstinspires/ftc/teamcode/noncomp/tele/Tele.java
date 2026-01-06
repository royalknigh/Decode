package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;

@TeleOp(name="Competition TeleOp", group="Iterative OpMode")
public class Tele extends OpMode {
    private enum State {INIT, INTAKE, LAUNCH}
    private State state;

    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LimelightController limelightController;
    private LaunchSystem launchSystem;

    private double hoodPosition = 0.5;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        limelightController = new LimelightController(
                hardwareMap.get(Limelight3A.class, "limelight"),
                servoConfig
        );
        launchSystem = new LaunchSystem(motorConfig, servoConfig);

        servoConfig.setInitPos();
        state = State.INIT;
    }

    @Override
    public void loop() {
        handleMovement();
        handleTracking();
        handleStateMachine();
        handleHood();
        displayTelemetry();
        telemetry.update();
    }

    private void handleMovement() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        motorConfig.setMotorPowers(
                (y + x + rx) / denominator,
                (y - x + rx) / denominator,
                (y - x - rx) / denominator,
                (y + x - rx) / denominator
        );
    }

    private void handleTracking() {
        if (gamepad1.a) limelightController.toggleTracking();
        limelightController.updateTracking();
    }

    private void handleStateMachine() {
        switch (state) {
            case INIT:
                // Flywheel is now automatically idling at 1000 ticks/sec
                if (gamepad1.left_trigger > 0) state = State.INTAKE;
                if (gamepad1.x) {
                    launchSystem.start();
                    state = State.LAUNCH;
                }
                // Optional: Kill motors completely if back button pressed
                if (gamepad1.back) launchSystem.fullStop();
                break;

            case INTAKE:
                motorConfig.intakeMotor.setPower(gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger : 0);
                if (gamepad1.x) {
                    launchSystem.start();
                    state = State.LAUNCH;
                }
                if (gamepad1.left_trigger <= 0.1) state = State.INIT;
                break;

            case LAUNCH:
                if (launchSystem.update()) {
                    state = State.INIT; // Returns to INIT, which keeps flywheel in idle
                }
                break;
        }
    }

    private void handleHood() {
        if (gamepad1.dpad_up) hoodPosition += 0.003;
        if (gamepad1.dpad_down) hoodPosition -= 0.003;
        hoodPosition = Range.clip(hoodPosition, 0, 1);
        servoConfig.hoodServo.setPosition(hoodPosition);
    }

    private void displayTelemetry() {
        telemetry.addLine("=== LAUNCHER STATUS ===");
        telemetry.addData("Status", launchSystem.isReady() ? "READY" : "WAITING");
        telemetry.addData("Velocity", "%.0f", launchSystem.getVelocity());
        telemetry.addData("Robot State", state);
        telemetry.addData("Hood Pos", "%.3f", hoodPosition);

        double dist = limelightController.getDistance();
        telemetry.addData("Limelight Dist", dist > 0 ? "%.1f in" : "NO TARGET");
    }
}