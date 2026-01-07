package org.firstinspires.ftc.teamcode.noncomp.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;

@Configurable
@TeleOp(name="Full Control Competition TeleOp", group="Iterative OpMode")
public class Tele extends OpMode {
    private enum State {INIT, INTAKE, LAUNCH}
    private State state;

    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;
    private double hoodPosition = 0.5;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        launchSystem = new LaunchSystem(motorConfig, servoConfig);
        servoConfig.setInitPos();
        state = State.INIT;
    }

    @Override
    public void loop() {
        handleMovement();
        handleStateMachine();
        handleHood();

        telemetry.addData("Launch Mode", launchSystem.getStatus());
        telemetry.addData("Flywheel Velocity", "%.0f", launchSystem.getVelocity());
        telemetry.addData("Hood Pos", "%.3f", hoodPosition);
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

                // X: High Power, Standard Interval (1.5s)
                if (gamepad1.x) {
                    launchSystem.start(LaunchSystem.HIGH_VELOCITY, 900);
                    state = State.LAUNCH;
                }
                // Y: Low Power, Rapid Interval (0.4s)
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.LOW_VELOCITY, 400);
                    state = State.LAUNCH;
                }
                break;

            case INTAKE:
                motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
                if (gamepad1.x) {
                    launchSystem.start(LaunchSystem.HIGH_VELOCITY, 900);
                    state = State.LAUNCH;
                }
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.LOW_VELOCITY, 700);
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
        if (gamepad1.dpad_up) hoodPosition += 0.003;
        if (gamepad1.dpad_down) hoodPosition -= 0.003;
        hoodPosition = Range.clip(hoodPosition, 0, 1);
        servoConfig.hoodServo.setPosition(hoodPosition);
    }
}