package org.firstinspires.ftc.teamcode.pedroPathing.examples;

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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Prima Iteratie", group = "Examples")
public class PedroAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;





    private final Pose StartPositon = new Pose(1.33, 94.95, Math.toRadians(0));
    private final Pose Chamber = new Pose(23.11, 86.96, Math.toRadians(0));

    private final Pose Spike = new Pose(62.85, 36.74, Math.toRadians(-90));
    //Control Point: 19,62  51,71
    private final Pose C1 = new Pose(12.63, 38.07, Math.toRadians(-90));
//19.62, 51.76
    private final Pose First = new Pose(7.15, 37.24, Math.toRadians(-90));
    private final Pose Bezier1 = new Pose(55.20, 27.93, Math.toRadians(-90));
    //Control Point: 80,81  26,77
    private final Pose C2 = new Pose(58.86, 43.23, Math.toRadians(-90));

    private final Pose Second = new Pose(8.31, 28.10, Math.toRadians(-90));

    private final Pose Third = new Pose(7.31, 22.78, Math.toRadians(-90));
    private final Pose C3 = new Pose(3.99, 22.78, Math.toRadians(-90));

    private Path SpikeCurve, RePosition1, Pickup;
    private PathChain Start, ElementPush1, ElementPush2;


    public void buildPaths(){
        Start=follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartPositon), new Point(Chamber)))
                .setLinearHeadingInterpolation(StartPositon.getHeading(), Chamber.getHeading())
                .build();

        SpikeCurve = new Path(new BezierCurve(new Point(Chamber), new Point(C1), new Point(Spike)));
        SpikeCurve.setLinearHeadingInterpolation(Chamber.getHeading(), Spike.getHeading());

        ElementPush1=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Spike), new Point(First)))
                .setLinearHeadingInterpolation(Spike.getHeading(), First.getHeading())
                .build();

        RePosition1 = new Path(new BezierCurve(new Point(First), new Point(C2), new Point(Bezier1)));
        RePosition1.setLinearHeadingInterpolation(First.getHeading(), Bezier1.getHeading());

        ElementPush2=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Bezier1), new Point(Second)))
                .setLinearHeadingInterpolation(Bezier1.getHeading(), Second.getHeading())
                .build();

        Pickup = new Path(new BezierCurve(new Point(Second), new Point(C3), new Point(Third)));
        Pickup.setLinearHeadingInterpolation(Second.getHeading(), Third.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Start);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(SpikeCurve, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(ElementPush1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(RePosition1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(ElementPush2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Pickup, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }


    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(StartPositon);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

