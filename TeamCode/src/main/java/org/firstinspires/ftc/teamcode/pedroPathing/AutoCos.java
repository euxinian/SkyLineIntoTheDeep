package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.opencv.ml.SVM.C;

import java.util.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Cos", group = "Examples")
public class AutoCos extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor armExtendMotor, armLiftMotor = null;
    private Servo Gheara;






    private final Pose StartPositon = new Pose(1.32, 95.92, Math.toRadians(90));
    private final Pose PreloadScore = new Pose(1.99, 139.67, Math.toRadians(135)); //2.32, 137.18
    private final Pose Sample1 = new Pose(47.02, 132.83, Math.toRadians(180));
    private final Pose A = new Pose(40.55, 100.8, Math.toRadians(180));
    private final Pose Score = new Pose(4.82, 140.00, Math.toRadians(135));
    private final Pose Sample2 = new Pose(47.31, 142.53, Math.toRadians(180));
    private final Pose B = new Pose(45.25, 117.55, Math.toRadians(180));

    private final Pose Sub = new Pose(60.83, 103.15, Math.toRadians(90));








    private Path Traj1, Traj2;
    private PathChain Line, Sybau, Sybau2, Park;




    public void buildPaths(){
        Line=follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartPositon), new Point(PreloadScore)))
                .setLinearHeadingInterpolation(StartPositon.getHeading(), PreloadScore.getHeading())
                .build();

        Traj1 = new Path(new BezierCurve(new Point(PreloadScore), new Point(A), new Point(Sample1)));
        Traj1.setLinearHeadingInterpolation(PreloadScore.getHeading(), Sample1.getHeading());

        Sybau=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Sample1), new Point(Score)))
                .setLinearHeadingInterpolation(Sample1.getHeading(), Score.getHeading())
                .build();

        Traj2 = new Path(new BezierCurve(new Point(Score), new Point(B), new Point(Sample2)));
        Traj2.setLinearHeadingInterpolation(Score.getHeading(), Sample2.getHeading());

        Sybau2=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Sample2), new Point(Score)))
                .setLinearHeadingInterpolation(Sample2.getHeading(), Score.getHeading())
                .build();
        Park=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Score), new Point(Sub)))
                .setLinearHeadingInterpolation(Score.getHeading(), Sub.getHeading())
                .build();





    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Gheara.setPosition(0.9);
                armLiftMotor.setTargetPosition(-135);
                armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLiftMotor.setPower(0.55);
                armExtendMotor.setTargetPosition(1500); //2100
                armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtendMotor.setPower(0.5);
                follower.followPath(Line);
                setPathState(1);
                //Duce primul sample la cos (preload)
                break;

            case 1:
                if (!follower.isBusy() && !armLiftMotor.isBusy() && !armExtendMotor.isBusy()) {
                    armLiftMotor.setTargetPosition(30);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLiftMotor.setPower(0.45);
                    Gheara.setPosition(0.85);
                    setPathState(2);
                    //Arunca Sample-ul in cos
                }
                break;

            case 2:

                if (!follower.isBusy() && !armLiftMotor.isBusy()) {
                    armLiftMotor.setTargetPosition(-100);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLiftMotor.setPower(0.60);

                    setPathState(3);
                }
                break;
            case 3:

                if (!follower.isBusy() && !armLiftMotor.isBusy()) {
                    follower.followPath(Traj1, true);
                    armExtendMotor.setTargetPosition(100);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    setPathState(4);
                    //Case 2 + Case 3: duc la poziie normala sa nu pice bratul
                }
                break;

            case 4:
                if (!follower.isBusy() && !armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    armLiftMotor.setTargetPosition(950);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLiftMotor.setPower(0.55);
                    armExtendMotor.setTargetPosition(530);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && !armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    Gheara.setPosition(0.9);
                    if(Gheara.getPosition()==0.9) {
                        armLiftMotor.setTargetPosition(30);
                        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armLiftMotor.setPower(0.55);
                        setPathState(6);
                    }

                }
                break;
            case 6:
                if (!armLiftMotor.isBusy()) {
                    armExtendMotor.setTargetPosition(1500);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Sybau, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    Gheara.setPosition(0.85);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    armExtendMotor.setTargetPosition(100);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    follower.followPath(Traj2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && !armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    armLiftMotor.setTargetPosition(950);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLiftMotor.setPower(0.55);
                    armExtendMotor.setTargetPosition(500);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && !armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    Gheara.setPosition(0.9);
                    if(Gheara.getPosition()==0.9) {
                        armLiftMotor.setTargetPosition(30);
                        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armLiftMotor.setPower(0.55);
                        setPathState(12);
                    }

                }
                break;
            case 12:
                if (!armLiftMotor.isBusy()) {
                    armExtendMotor.setTargetPosition(1500);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(Sybau2, true);
                    setPathState(14);
                }
            case 14:
                if (!follower.isBusy()) {
                    Gheara.setPosition(0.85);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy() && Gheara.getPosition()==0.85) {
                    armExtendMotor.setTargetPosition(0);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.5);
                    armLiftMotor.setTargetPosition(0);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLiftMotor.setPower(0.55);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Park, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {

                    setPathState(-1);
                }
                break;








        }


    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(StartPositon);
        buildPaths();

        Gheara = hardwareMap.get(Servo .class, "claw_servo");
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        armLiftMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}

