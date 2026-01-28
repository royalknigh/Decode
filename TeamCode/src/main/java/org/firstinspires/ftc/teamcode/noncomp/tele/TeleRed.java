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
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

@Configurable
@TeleOp(name="TeleOp Red", group="Iterative OpMode")
public class TeleRed extends OpMode {

    public static LimelightController.Alliance alliance = LimelightController.Alliance.BLUE;

    private enum State {INIT, INTAKE, LAUNCH, EJECT}
    private State state;
    private boolean lastB = false;
    private boolean lastA = false;
    private double hoodPosition = 0.5;

    public static int lowTime= 300;
    public static int highTime =400;

    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;
    private LimelightController limelightController;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        launchSystem = new LaunchSystem(motorConfig, servoConfig);
        limelightController = new LimelightController(hardwareMap.get(Limelight3A.class, "limelight"), servoConfig);
        limelightController.setAlliance(LimelightController.Alliance.RED);
        servoConfig.setInitPos();
        state = State.INIT;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        if (gamepad1.a && !lastA) {
            limelightController.toggleTracking();
        }
        lastA = gamepad1.a;

        limelightController.updateTracking();
        handleMovement();
        handleStateMachine();
        handleHood();
        spit();
        manualTurret();
        if(gamepad1.right_bumper) launchSystem.fullStop();

        telemetry.addData("State", state);
        telemetry.addData("Velocity", motorConfig.launchMotor1.getVelocity());
        telemetry.addData("Distance", limelightController.getDistance());
        telemetry.addData("alliance ", alliance);
        telemetry.update();
    }

    private void handleMovement() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x / 2.0;
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
                    launchSystem.start(LaunchSystem.highVelocity, highTime);
                    state = State.LAUNCH;
                }
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.lowVelocity, lowTime);
                    state = State.LAUNCH;
                }
                if (gamepad1.right_trigger > 0.1) {
                    state = State.EJECT;
                }
                break;

            case INTAKE:
                motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
                if (gamepad1.x) {
                    launchSystem.start(LaunchSystem.highVelocity, highTime);
                    state = State.LAUNCH;
                }
                if (gamepad1.y) {
                    launchSystem.start(LaunchSystem.lowVelocity, lowTime);
                    state = State.LAUNCH;
                }
                if (gamepad1.left_trigger <= 0.1) {
                    motorConfig.intakeMotor.setPower(0);
                    state = State.INIT;
                }
                break;

            case LAUNCH:
                if (launchSystem.update()) {
                    state = State.INIT;
                }
                break;
            case EJECT:
                if (gamepad1.right_trigger > 0.1) {
                    motorConfig.intakeMotor.setPower(-gamepad1.right_trigger);
                    servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
                } else {
                    state = State.INIT;
                    motorConfig.intakeMotor.setPower(0);
                }
        }
    }

    private void handleHood() {
        double x = limelightController.getDistance();
        if (x < 90) {
            hoodPosition = -0.006 * x + 0.946667;
        } else {
            hoodPosition = 0.95;
        }

        if(gamepad1.dpad_up) hoodPosition += 0.002;
        if(gamepad1.dpad_down) hoodPosition -= 0.002;

        hoodPosition = Range.clip(hoodPosition, 0, 0.98);
        servoConfig.hoodServo.setPosition(hoodPosition);
    }
    public void spit(){

    }
    public void manualTurret() {
        if (!limelightController.isTrackingEnabled()) {
            if (gamepad1.dpad_right) {
                servoConfig.leftTurretServo.setPower(1);
                servoConfig.rightTurretServo.setPower(1);
            } else if (gamepad1.dpad_left) {
                servoConfig.leftTurretServo.setPower(-1);
                servoConfig.rightTurretServo.setPower(-1);

            } else {
                servoConfig.leftTurretServo.setPower(0);
                servoConfig.rightTurretServo.setPower(0);
            }
        }
    }
}