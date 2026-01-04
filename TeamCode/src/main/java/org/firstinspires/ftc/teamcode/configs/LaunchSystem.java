package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

public class LaunchSystem {
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private ElapsedTime launchTimer = new ElapsedTime();
    private int launchStep = 0;
    private boolean resetTimer = true;
    
    public LaunchSystem(MotorConfig motorConfig, ServoConfig servoConfig) {
        this.motorConfig = motorConfig;
        this.servoConfig = servoConfig;
    }
    
    public void start() {
        launchTimer.reset();
        launchStep = 0;
        resetTimer = true;
    }
    
    public boolean update() { // Returns true when complete
        if(resetTimer){
            resetTimer = false;
            motorConfig.launchMotor1.setPower(1);
            motorConfig.launchMotor2.setPower(1);
            launchTimer.reset();
        }

        if (launchStep == 0 && launchTimer.milliseconds() >= 2000) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_MID);
            launchStep = 1;
            launchTimer.reset();
        } else if (launchStep == 1 && launchTimer.milliseconds() >= 500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            motorConfig.intakeMotor.setPower(0.8);
            launchStep = 2;
            launchTimer.reset();
        } else if (launchStep == 2 && launchTimer.milliseconds() >= 1500) {
            motorConfig.intakeMotor.setPower(0);
            servoConfig.launchServo.setPosition(ServoConstants.launch_MID);
            launchStep = 3;
            launchTimer.reset();
        } else if (launchStep == 3 && launchTimer.milliseconds() >= 1500) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_PUSH);
            launchStep = 4;
            launchTimer.reset();
        }
        
        if (launchStep >= 4 && launchTimer.milliseconds() >= 500) {
            resetTimer = true;
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            return true; // Launch complete
        }
        return false; // Still launching
    }
    
    public void stop() {
        motorConfig.launchMotor1.setPower(0);
        motorConfig.launchMotor2.setPower(0);
    }
}