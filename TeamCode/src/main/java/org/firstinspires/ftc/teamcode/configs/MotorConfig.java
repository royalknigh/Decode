package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorConfig {

    public static double lP = 0, lI = 0, lD = 0, lF = 0;
    public PIDFController liftPID;
    public static int intTargetPosition = 0;
    private boolean isSlideDown;

    public static DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor,
            backRightMotor, launchMotor1, launchMotor2, liftLeftMotor, liftRightMotor, intakeMotor;

    public MotorConfig(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "fr");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "br");

        launchMotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lm1");
        launchMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lm2");
        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "im");

//        liftLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftLeftMotor");
//        liftRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftRightMotor");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        launchMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        launchMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launchMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launchMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


//        liftLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        liftLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        liftLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        liftRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        liftRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        liftRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType configlaunchMotor1 = launchMotor1.getMotorType().clone();
        configlaunchMotor1.setAchieveableMaxRPMFraction(1.0);
        launchMotor1.setMotorType(configlaunchMotor1);

        MotorConfigurationType configlaunchMotor2 = launchMotor2.getMotorType().clone();
        configlaunchMotor2.setAchieveableMaxRPMFraction(1.0);
        launchMotor2.setMotorType(configlaunchMotor2);

//        MotorConfigurationType configLiftLeftMotor = liftLeftMotor.getMotorType().clone();
//        configLiftLeftMotor.setAchieveableMaxRPMFraction(1.0);
//        liftLeftMotor.setMotorType(configLiftLeftMotor);
//
//        MotorConfigurationType configLiftRightMotor = liftLeftMotor.getMotorType().clone();
//        configLiftRightMotor.setAchieveableMaxRPMFraction(1.0);
//        liftLeftMotor.setMotorType(configLiftRightMotor);

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

        MotorConfigurationType configIntakeMotor = intakeMotor.getMotorType().clone();
        configIntakeMotor.setAchieveableMaxRPMFraction(1.0);
        intakeMotor.setMotorType(configIntakeMotor);
    }


    public void mecanumDrive(double y, double x, double rx, double speedDivider) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator / speedDivider;
        double backLeftPower = (y - x + rx) / denominator / speedDivider;
        double frontRightPower = (y - x - rx) / denominator / speedDivider;
        double backRightPower = (y + x - rx) / denominator / speedDivider;

        setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

//    public void setLiftPID() {
//        liftPID.setTargetPosition(intTargetPosition);
//        liftPID.updatePosition(liftRightMotor.getCurrentPosition());
//
//        double liftPower = liftPID.run();
//
//        if (Math.abs(liftRightMotor.getCurrentPosition() - intTargetPosition) < 10)
//            liftPower = 0;
//
//        if (liftRightMotor.getCurrentPosition() < 5 && liftRightMotor.getVelocity() < 0.05 && intTargetPosition ==0) {
//            liftRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            liftLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//            liftRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            liftLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//            isSlideDown = true;
//        }
//
//        if (intTargetPosition == 0 && isSlideDown){
//            liftRightMotor.setPower(0);
//            liftLeftMotor.setPower(0);
//        }
//
//        else {
//            liftRightMotor.setPower(liftPower);
//            liftLeftMotor.setPower(liftPower);
//        }
//
//    }

//    public void updatePIDFController() {
//        PIDFCoefficients intCoefficients = new PIDFCoefficients(lP, lI, lD, lF);
//        liftPID = new PIDFController(intCoefficients);
//    }
}