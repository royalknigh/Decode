package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorConfig {

    public static DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor,
            backRightMotor, launchMotor, liftLeftMotor, liftRightMotor, inatkeMotor;
    public MotorConfig(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRightMotor");

        launchMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "launchMotor");
        inatkeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor");

        liftLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftLeftMotor");
        liftRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftRightMotor");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        inatkeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        inatkeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        launchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        inatkeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        inatkeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        liftLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType configLaunchMotor = launchMotor.getMotorType().clone();
        configLaunchMotor.setAchieveableMaxRPMFraction(1.0);
        launchMotor.setMotorType(configLaunchMotor);

        MotorConfigurationType configLiftLeftMotor = liftLeftMotor.getMotorType().clone();
        configLiftLeftMotor.setAchieveableMaxRPMFraction(1.0);
        liftLeftMotor.setMotorType(configLiftLeftMotor);

        MotorConfigurationType configLiftRightMotor = liftLeftMotor.getMotorType().clone();
        configLiftRightMotor.setAchieveableMaxRPMFraction(1.0);
        liftLeftMotor.setMotorType(configLiftRightMotor);

        MotorConfigurationType configFrontLeftMotor = frontLeftMotor.getMotorType().clone();
        configFrontLeftMotor.setAchieveableMaxRPMFraction(1.0);
        frontLeftMotor.setMotorType(configFrontLeftMotor);

        MotorConfigurationType configBackLeftMotor = backLeftMotor.getMotorType().clone();
        configBackLeftMotor.setAchieveableMaxRPMFraction(1.0);
        backLeftMotor.setMotorType(configBackLeftMotor);

        MotorConfigurationType configFrontRightMotor = frontRightMotor.getMotorType().clone();
        configFrontRightMotor.setAchieveableMaxRPMFraction(1.0);
        frontRightMotor.setMotorType(configFrontRightMotor);

        MotorConfigurationType configBackRightMotor = backRightMotor.getMotorType().clone();
        configBackRightMotor.setAchieveableMaxRPMFraction(1.0);
        backRightMotor.setMotorType(configBackRightMotor);

        MotorConfigurationType configIntakeMotor = liftLeftMotor.getMotorType().clone();
        configIntakeMotor.setAchieveableMaxRPMFraction(1.0);
        liftLeftMotor.setMotorType(configIntakeMotor);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
