package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "GenPath", group = "Examples")
public class GeneratedPath extends OpMode {

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState;

        private final Pose[] poses = {
                new Pose(8.938, 63.393, Math.toRadians(0)),
                new Pose(34.759, 63.393, Math.toRadians(0)),
                new Pose(34.759, 36.579, Math.toRadians(0)),
                new Pose(59.090, 36.745, Math.toRadians(0)),
                new Pose(58.924, 22.841, Math.toRadians(0)),
                new Pose(18.207, 22.841, Math.toRadians(0)),
                new Pose(58.924, 12.414, Math.toRadians(0)),
                new Pose(17.545, 12.579, Math.toRadians(0)),
                new Pose(58.924, 3.972, Math.toRadians(0)),
                new Pose(17.545, 4.138, Math.toRadians(0)),
                new Pose(18.041, 23.834, Math.toRadians(0)),
                new Pose(5.628, 23.834, Math.toRadians(0)),
                new Pose(34.759, 63.393, Math.toRadians(0)),
                new Pose(41.379, 45.517, Math.toRadians(0)),
                new Pose(72.166, 49.655, Math.toRadians(0))
        };

        private PathChain[] paths;

        public void buildPaths() {
                paths = new PathChain[poses.length - 1];
                for (int i = 0; i < poses.length - 1; i++) {
                        paths[i] = follower.pathBuilder()
                                .addPath(new BezierLine(new Point(poses[i]), new Point(poses[i + 1])))
                                .setTangentHeadingInterpolation()
                                .build();
                }
        }

        public void autonomousPathUpdate() {
                if (pathState < paths.length) {
                        follower.followPath(paths[pathState]);
                        pathState++;
                } else {
                        setPathState(-1);
                }
        }

        @Override
        public void loop() {
                follower.update();
                autonomousPathUpdate();
                telemetry.addData("path state", pathState);
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
                follower.setStartingPose(poses[0]);
                buildPaths();
        }

        @Override
        public void start() {
                opmodeTimer.resetTimer();
                setPathState(0);
        }

        public void setPathState(int pState) {
                pathState = pState;
                pathTimer.resetTimer();
        }

        @Override
        public void stop() {}
}
