package org.firstinspires.ftc.teamcode.noncomp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.noncomp.tele.Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Blue Short")
public class AutoBlue extends OpMode {
    private LimelightController limelightController;
    private Follower follower;
    private Timer pathTimer;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;
    private Tele teleOp;

    private int pathState = 0;
    private int interval = 400;
    private boolean lastB = false;
    public static LimelightController.Alliance alliance = LimelightController.Alliance.BLUE;
    private boolean calibrated = false;
    // Blue Side Poses
    private final Pose startPose = new Pose(27, 117, Math.toRadians(140));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(140));
    private final Pose fisrtLinePose = new Pose(48, 80, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(20, 80, Math.toRadians(180));
//    private final Pose openGatePose = new Pose(13, 68, Math.toRadians(90));
    private final Pose secondLinePose = new Pose(48, 58, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(10, 58, Math.toRadians(180));
    private final Pose thirdLinePose = new Pose(48, 34, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(10, 34, Math.toRadians(180));

    private PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2,
            openGate, score2, alignRow3, pickupRow3, score3, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.4, () -> launchSystem.start(LaunchSystem.lowVelocity, interval))
                .build();

//        openGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pose, new Pose(29, 48), openGatePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), openGatePose.getHeading()).build();

        alignRow1 = follower.pathBuilder().addPath(new BezierLine(scorePose, fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading()).build();

        pickupRow1 = follower.pathBuilder().addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading()).build();

        score1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.8))
                .addParametricCallback(0.2, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.4, () -> limelightController.toggleTracking()) //0.4
                .addParametricCallback(0.8, () -> launchSystem.start(LaunchSystem.lowVelocity, interval))
                .build();

        alignRow2 = follower.pathBuilder().addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading()).build();

        pickupRow2 = follower.pathBuilder().addPath(new BezierLine(secondLinePose, pickup2Pose))
                .addParametricCallback(0, () -> follower.setMaxPower(0.4))
//                .addParametricCallback(0.95, () -> motorConfig.intakeMotor.setPower(0))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading()).build();

        score2 = follower.pathBuilder().addPath(new BezierCurve(pickup2Pose, new Pose(48, 55), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.6))
                .addParametricCallback(0.1, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.5, () -> limelightController.toggleTracking()) //0.5
                .addParametricCallback(0.8, () -> launchSystem.start(LaunchSystem.lowVelocity, interval))
                .build();

        alignRow3 = follower.pathBuilder().addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading()).build();

        pickupRow3 = follower.pathBuilder().addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .addParametricCallback(0, () -> follower.setMaxPower(0.4))
//                .addParametricCallback(0.9, () -> motorConfig.intakeMotor.setPower(0))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading()).build();

        score3 = follower.pathBuilder().addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.7))
                .addParametricCallback(0.1, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.6, () -> limelightController.toggleTracking()) //0.6
                .addParametricCallback(0.8, () -> launchSystem.start(LaunchSystem.lowVelocity, interval))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    private void handleStateTransition(int nextState, PathChain nextPath) {
        if (!follower.isBusy()) {
            // launchSystem.update() returns true when the firing sequence is done
            if (launchSystem.update()) {
                limelightController.toggleTracking();
                if (nextPath != null) follower.followPath(nextPath);
                setPathState(nextState);
            }
        }
    }

    private void performPickup(PathChain pickupPath, int nextState) {
        if (!follower.isBusy()) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            motorConfig.intakeMotor.setPower(1);
            follower.setMaxPower(0.7);
            follower.followPath(pickupPath);
            setPathState(nextState);
        }
    }

    private void returnToScore(PathChain scorePath, int nextState) {
        if (!follower.isBusy()) {
            follower.setMaxPower(1.0);
            follower.followPath(scorePath);
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                limelightController.toggleTracking();
                setPathState(1);
                break;
            case 1:
                handleStateTransition(2, alignRow1);
                break;
            case 2:
                performPickup(pickupRow1, 3);
                break;
            case 3:
                returnToScore(score1, 4);
                break;
            case 4:
                handleStateTransition(5, alignRow2);
                break;
            case 5:
                performPickup(pickupRow2, 6);
                break;
            case 6:
                returnToScore(score2, 7);
                break;
            case 7:
                handleStateTransition(8, alignRow3);
                break;
            case 8:
                performPickup(pickupRow3, 9);
                break;
            case 9:
                returnToScore(score3, 10);
                break;
            case 10:
                handleStateTransition(-1, park);
                follower.setMaxPower(1);
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(openGate);
                    setPathState(6);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        launchSystem = new LaunchSystem(motorConfig, servoConfig);
        limelightController = new LimelightController(hardwareMap.get(Limelight3A.class, "limelight"), servoConfig);
        limelightController.getLimelight().pipelineSwitch(5);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }



    @Override
    public void loop() {
        follower.update();
        limelightController.updateTracking();
        handleHood();

        // Essential: Keep the launcher state machine running independently of paths
        launchSystem.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Calibrating: ", calibrated);
        telemetry.update();
    }

//    private boolean calibrateTurret(){
//        if(!calibrated) {
//            servoConfig.leftTurretServo.setPower(1);
//            servoConfig.rightTurretServo.setPower(1);
//            calibrated=true;
//        }
//        else {
//            servoConfig.leftTurretServo.setPower(0);
//            servoConfig.rightTurretServo.setPower(0);
//            calibrated=false;
//        }
//        return calibrated;
//    }


    /*private void handleHood() {
        double x = limelightController.getDistance();
        double pos = -0.006 * x + 0.946667;
        servoConfig.hoodServo.setPosition(Range.clip(pos, 0, 1));
    }*/

    public void handleHood() {
        double x = limelightController.getDistance();
        double hoodPosition;
        if (x < 90) {
            hoodPosition = -0.005 * x + 1.02667;//new
//            hoodPosition = -0.006 * x + 0.946667;//old
        } else {
            hoodPosition = 0.95;
        }

        if(gamepad1.dpad_up) hoodPosition += 0.002;
        if(gamepad1.dpad_down) hoodPosition -= 0.002;

        hoodPosition = Range.clip(hoodPosition, 0, 0.98);
        servoConfig.hoodServo.setPosition(hoodPosition);
    }

    @Override public void init_loop() {
        motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
    }
    @Override public void start() {
        setPathState(0);
        launchSystem.setDualVelocity(900);
    }
}