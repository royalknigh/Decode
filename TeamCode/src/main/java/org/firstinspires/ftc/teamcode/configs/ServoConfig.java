package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ServoConstants;
public class ServoConfig {
    public static Servo leftTurretServo, rightTurretServo, launchServo, hoodServo;

    public ServoConfig(HardwareMap hardwareMap){

        leftTurretServo = hardwareMap.get(Servo.class,  "leftTurret");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurret");
        launchServo = hardwareMap.get(Servo.class, "launch");
        hoodServo = hardwareMap.get(Servo.class, "hood");
    }
    public void setServoPos(double lT, double rT, double l){
        leftTurretServo.setPosition(lT);
        rightTurretServo.setPosition(rT);
        launchServo.setPosition(l);
    }
    public void setInitPos(){
        leftTurretServo.setPosition(ServoConstants.turret_INIT);
        rightTurretServo.setPosition(ServoConstants.turret_INIT);
        launchServo.setPosition(ServoConstants.launch_INIT);
        hoodServo.setPosition(ServoConstants.hood_INIT);
    }
}
