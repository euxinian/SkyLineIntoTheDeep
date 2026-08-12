package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Square", group = "Examples")
public class Square extends OpMode {
    private Follower follower;

    private final Pose downLeft = new Pose(0,0, Math.toRadians(0));
    private final Pose downRight = new Pose(24, 0, Math.toRadians(90));
    private final Pose upRight = new Pose(24, 24, Math.toRadians(90));
    private final Pose upLeft = new Pose(0, 24, Math.toRadians(90));

    private PathChain square;

    private Telemetry telemetryA;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(square, true);
        }

        follower.telemetryDebug(telemetryA);
    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(downLeft);

        square = follower.pathBuilder()
                .addPath(new BezierLine(new Point(downLeft), new Point(downRight)))
                .setLinearHeadingInterpolation(downLeft.getHeading(), downRight.getHeading())
                .addPath(new BezierLine(new Point(downRight), new Point(upRight)))
                .setLinearHeadingInterpolation(downRight.getHeading(), upRight.getHeading())
                .addPath(new BezierLine(new Point(upRight), new Point(upLeft)))
                .setLinearHeadingInterpolation(upRight.getHeading(), upLeft.getHeading())
                .addPath(new BezierLine(new Point(upLeft), new Point(downLeft)))
                .setLinearHeadingInterpolation(upLeft.getHeading(), downLeft.getHeading())
                .build();

        follower.followPath(square);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}