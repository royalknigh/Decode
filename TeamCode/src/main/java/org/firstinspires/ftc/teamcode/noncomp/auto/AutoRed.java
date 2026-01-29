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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.noncomp.tele.Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Red Short")
public class AutoRed extends OpMode {
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
    public static LimelightController.Alliance alliance = LimelightController.Alliance.RED;

    // Mirrored Red Poses
    private final Pose startPose = new Pose(117, 120, Math.toRadians(37));
    private final Pose scorePose = new Pose(85, 89, Math.toRadians(30));
    private final Pose fisrtLinePose = new Pose(96, 82, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(124, 82, Math.toRadians(0));
    private final Pose secondLinePose = new Pose(96, 59, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(131, 59, Math.toRadians(0));
    private final Pose thirdLinePose = new Pose(96, 35, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(131, 35, Math.toRadians(0));

    private PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2,
            pickupRow2, score2, alignRow3, pickupRow3, score3, park;

    public void buildPaths() {
        // Preload: Start spinning wheels immediately
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.3, () -> launchSystem.start(LaunchSystem.lowVelocity, interval))
                .build();

        alignRow1 = follower.pathBuilder().addPath(new BezierLine(scorePose, fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading()).build();

        pickupRow1 = follower.pathBuilder().addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading()).build();

        // Score 1: Intake outtake early, spin up launcher mid-path
        score1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.8))
                .addParametricCallback(0.3, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.3, () -> limelightController.toggleTracking())
                .addParametricCallback(0.5, () -> {
                    launchSystem.start(LaunchSystem.lowVelocity, interval);
                })
                .build();

        alignRow2 = follower.pathBuilder().addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading()).build();

        pickupRow2 = follower.pathBuilder().addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading()).build();

        score2 = follower.pathBuilder().addPath(new BezierCurve(pickup2Pose, new Pose(90, 60), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.7))
                .addParametricCallback(0.3, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.6, () -> limelightController.toggleTracking())
                .addParametricCallback(0.7, () -> {
                    launchSystem.start(LaunchSystem.lowVelocity, interval);
                })
                .addParametricCallback(0.8, () -> motorConfig.intakeMotor.setPower(0))
                .build();

        alignRow3 = follower.pathBuilder().addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading()).build();

        pickupRow3 = follower.pathBuilder().addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading()).build();

        score3 = follower.pathBuilder().addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> motorConfig.intakeMotor.setPower(0.7))
                .addParametricCallback(0.2, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0.8, () -> limelightController.toggleTracking())
                .addParametricCallback(0.8, () -> {
                    launchSystem.start(LaunchSystem.lowVelocity, interval);
                })
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    private void performLaunchSequence(int nextState, PathChain nextPath) {
        // We wait for the path to end. The launch was triggered during the path via callback.
        if (!follower.isBusy()) {
            // update() must be called to handle the internal launch timer/servo trigger
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
            motorConfig.intakeMotor.setPower(0.9);
            follower.setMaxPower(0.75);
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
                performLaunchSequence(2, alignRow1);
                break;
            case 2:
                performPickup(pickupRow1, 3);
                break;
            case 3:
                returnToScore(score1, 4);
                break;
            case 4:
                performLaunchSequence(5, alignRow2);
                break;
            case 5:
                performPickup(pickupRow2, 6);
                break;
            case 6:
                returnToScore(score2, 7);
                break;
            case 7:
                performLaunchSequence(8, alignRow3);
                break;
            case 8:
                performPickup(pickupRow3, 9);
                break;
            case 9:
                returnToScore(score3, 10);
                break;
            case 10:
                performLaunchSequence(-1, park);
                follower.setMaxPower(1);
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
        limelightController.getLimelight().pipelineSwitch(6);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
        motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
        lastB = gamepad1.b;
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }


    @Override
    public void loop() {
        follower.update();
        limelightController.updateTracking();

        handleHood();

        // Essential: keeps the LaunchSystem state machine running
        launchSystem.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    @Override public void start() {
        setPathState(0);
        launchSystem.setDualVelocity(900);
    }

    public void handleHood() {
        double x = limelightController.getDistance();
        double hoodPosition;
        if (x < 90) {
            hoodPosition = -0.005 * x + 1.02667;
        } else {
            hoodPosition = 0.95;
        }

        if(gamepad1.dpad_up) hoodPosition += 0.002;
        if(gamepad1.dpad_down) hoodPosition -= 0.002;

        hoodPosition = Range.clip(hoodPosition, 0, 0.98);
        servoConfig.hoodServo.setPosition(hoodPosition);
    }
}