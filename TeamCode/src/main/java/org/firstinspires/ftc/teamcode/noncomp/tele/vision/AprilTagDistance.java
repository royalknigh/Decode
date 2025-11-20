package org.firstinspires.ftc.teamcode.noncomp.tele.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class AprilTagDistance extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;
    private double distance;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight3A.pipelineSwitch(5);  // #tag 11

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {`
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("distance: ", distance);
            telemetry.addData("Tx: ", llResult.getTx());
            telemetry.addData("Ty: ", llResult.getTy());
            telemetry.addData("Ta: ", llResult.getTa());

        }
    }

    public double getDistanceFromTag(double ta) {
        double scale = 500;
        double distance = (scale / ta);
        return distance;
    }


}
