package org.firstinspires.ftc.teamcode.noncomp.auto;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import static org.firstinspires.ftc.teamcode.configs.MotorConfig.launchMotor1;
import static org.firstinspires.ftc.teamcode.configs.MotorConfig.launchMotor2;
import static org.firstinspires.ftc.teamcode.configs.ServoConfig.launchServo;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.noncomp.tele.Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends OpMode {
    private LimelightController limelightController;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private int pathState = 0;
    private ElapsedTime launchTimer = new ElapsedTime();
//    private MotorConfig motorConfig;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(63, 115, Math.toRadians(150));
    private final Pose fisrtLinePose = new Pose(55, 96, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(29, 96, Math.toRadians(180));
    private final Pose secondLinePose = new Pose(55, 72, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 72, Math.toRadians(180));
    private final Pose thirdLinePose = new Pose(55, 45, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(24, 45, Math.toRadians(180));
    Servo launchServo = null;
    public PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2, score2, alignRow3, pickupRow3, score3;

    private int launchCount = 0;
    private int launchStep = 0;
    boolean resetTimer = true;



    public void buildPaths() {

        scorePreload = follower
                .pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(75.000, 83.000), scorePose))
//                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        alignRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(scorePose,new Pose(58.000, 96.000), fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading())
                .build();

        pickupRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.4, () -> follower.setMaxPower(0.7))
                .build();

        score1 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        alignRow2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();

        pickupRow2 = follower
                .pathBuilder()
                .addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading())
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,new Pose(60.000, 60.000),  scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        alignRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading())
                .build();

        pickupRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading())
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, new Pose(50.000, 55.000), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
//                motorConfig.launchMotor1.setPower(1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    launch(2, alignRow1);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    launchServo.setPosition(ServoConstants.launch_INIT);
                    follower.followPath(pickupRow1);
                    follower.setMaxPower(0.7);
                    motorConfig.intakeMotor.setPower(1);
                    launchMotor1.setPower(0);
                    launchMotor2.setPower(0);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(score1);
                    follower.setMaxPower(1);
                    motorConfig.intakeMotor.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launch(5, alignRow2);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    motorConfig.intakeMotor.setPower(1);
                    follower.followPath(pickupRow2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    motorConfig.intakeMotor.setPower(0);
                    follower.followPath(score2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    launch(8, alignRow3);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    motorConfig.intakeMotor.setPower(1);
                    follower.followPath(pickupRow3);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    motorConfig.intakeMotor.setPower(0);
                    follower.followPath(score3);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy())
                    launch(-1, score3);
                break;
        }
    }

    public void launch(int pathState, PathChain path){
        if(resetTimer){
            resetTimer = false;
            launchMotor1.setPower(1);
            launchMotor2.setPower(1);
            launchTimer.reset();
        }

        if (launchCount >= 2) {
            follower.followPath(path);
            resetTimer = true;
            launchServo.setPosition(ServoConstants.launch_INIT);
            setPathState(pathState);
            return;
        }
        if (launchStep == 0 && launchTimer.milliseconds() >= 500) {
            motorConfig.intakeMotor.setPower(0.7);
//            launchServo.setPosition(ServoConstants.launch_INIT);
            launchStep = 1;
            launchTimer.reset();
        } else if (launchStep == 1 && launchTimer.milliseconds() >= 2500) {
            launchServo.setPosition(ServoConstants.launch_PUSH);
            launchStep=2;
//            motorConfig.intakeMotor.setPower(0);

        }
    }
    public void setPathState(int state){
        pathState=state;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("motorPower", launchMotor1.getPower());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        limelightController.toggleTracking();
        motorConfig = new MotorConfig(hardwareMap);
        DcMotor launchMotor1 = null;
        DcMotor launchMotor2 = null;
        launchMotor1 = hardwareMap.get(DcMotor.class, "lm1");
        launchMotor2 = hardwareMap.get(DcMotor.class, "lm2");
        launchMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launchServo = hardwareMap.get(Servo.class, "ls");
        launchServo.setDirection(Servo.Direction.REVERSE);
        launchServo.setPosition(ServoConstants.launch_INIT);
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        limelightController.updateTracking();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
