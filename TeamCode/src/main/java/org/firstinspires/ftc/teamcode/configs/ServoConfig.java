package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ServoConstants;
public class ServoConfig {
    public static Servo  launchServo, hoodServo;
    public static CRServo leftTurretServo, rightTurretServo;

    public ServoConfig(HardwareMap hardwareMap){

        leftTurretServo = hardwareMap.get(CRServo.class,  "leftTurret");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurret");
        launchServo = hardwareMap.get(Servo.class, "ls");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        launchServo.setDirection(Servo.Direction.FORWARD);

    }
    public void setServoPos(double lT, double rT, double l){
        launchServo.setPosition(l);
    }
    public void setInitPos(){
        launchServo.setPosition(ServoConstants.launch_INIT);
        hoodServo.setPosition(ServoConstants.hood_INIT);
    }
}
