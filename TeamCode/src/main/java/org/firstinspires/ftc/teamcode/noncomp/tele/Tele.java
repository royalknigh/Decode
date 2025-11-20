/* Copyright (c) 2017 FIRST. All rights reserved.
 * [License details omitted for brevity, same as provided]
 */
package org.firstinspires.ftc.teamcode.noncomp.tele;
// cod smecher
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
@Disabled
public class Tele extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();
    private enum State {INIT, INTAKE, LAUNCH, LIFT}
    private State state = State.INIT;
    private final int fraction = 1;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private boolean empty = false;

    private int launchCount = 0;
    private int launchStep = 0;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        servoConfig.setInitPos();
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
        stateMachine();
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

    public void stateMachine() {
        switch (state) {
            case INIT: {
                motorConfig.launchMotor.setPower(0);
                motorConfig.inatkeMotor.setPower(0);
                if(gamepad1.left_trigger>0)
                    state = State.INTAKE;
                break;
            }
            case INTAKE: {
                if (gamepad1.left_trigger > 0)
                    motorConfig.inatkeMotor.setPower(0.5);
                if (gamepad1.a) {
                    state = State.LAUNCH;
                    resetLaunch();
                }
                break;
            }
            case LAUNCH: {
                motorConfig.launchMotor.setPower(1.0);
                motorConfig.inatkeMotor.setPower(0.6);

                if (launchCount >= 3) {
                    state = State.INIT;
                    break;
                }
                if (launchStep == 0 && launchTimer.milliseconds() >= 200) {
                    servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
                    launchStep = 1;
                    launchTimer.reset();
                } else if (launchStep == 1 && launchTimer.milliseconds() >= 300) {
                    servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
                    launchStep = 2;
                    launchTimer.reset();
                } else if (launchStep == 2 && launchTimer.milliseconds() >= 400) {
                    launchCount++;
                    launchStep = 0;
                    launchTimer.reset();
                }
                break;
            }
            case LIFT: {
                // TODO: IMPLEMENT LIFT
                break;
            }
        }
    }
    public void resetLaunch(){
        launchTimer.reset();
        launchCount = 0;
        launchStep = 0;
    }
}