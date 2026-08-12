package org.firstinspires.ftc.teamcode.pedroPathing;

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


@Autonomous(name = "Cos V2", group = "Examples")
public class AutoCosV2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor armExtendMotor, armLiftMotor, ascentMotorLeft, ascentMotorRight = null;
    private Servo Gheara;

    private long GhearaTimp; //Masurarea timpului pentru a lasa gheara sa prinda elemente de joc
    //Masuram momentul in care gheara este inchisa, dupa care robotul se misca DUPA 500ms



    //-------------------------------------------------------------------------------------\\
    private final Pose StartPositon = new Pose(1.32, 95.92, Math.toRadians(90));
    private final Pose PreloadScore = new Pose(1.99, 139.67, Math.toRadians(135));
    private final Pose Sample1 = new Pose(47.55 , 132.52, Math.toRadians(180));
    private final Pose A = new Pose(40.55, 100.8, Math.toRadians(180));
    private final Pose X = new Pose(28.93, 120.05, Math.toRadians(180));
    private final Pose Score = new Pose(3.99, 141.67, Math.toRadians(135)); //4.82, 140
    private final Pose Sample2 = new Pose(48.55, 141.83, Math.toRadians(180));
    private final Pose B = new Pose(45.25, 117.55, Math.toRadians(180));

    private final Pose Sub = new Pose(60.85, 104.42, Math.toRadians(90));



    //-----------------------------------------------------------------------------\\
    private Path Traj1, Traj2, Sybau;
    private PathChain Line, Sybau2, Park;
    //------------------------------------------------------------------------------\\




    public void buildPaths(){
        Line=follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartPositon), new Point(PreloadScore)))
                .setLinearHeadingInterpolation(StartPositon.getHeading(), PreloadScore.getHeading())
                .build();

        Traj1 = new Path(new BezierCurve(new Point(PreloadScore), new Point(A), new Point(Sample1)));
        Traj1.setLinearHeadingInterpolation(PreloadScore.getHeading(), Sample1.getHeading());

        Sybau = new Path(new BezierCurve(new Point(Sample1), new Point(X), new Point(Score)));
        Sybau.setLinearHeadingInterpolation(Sample1.getHeading(), Score.getHeading());
        //MAI ADAUGA UN PUNCT LA SYBAU DEOARECE DA PESTE AL 3-LEA SAMPLE

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
                Gheara.setPosition(0.92);
                MotorParameters(armLiftMotor, -135, 0.55);
                MotorParameters(armExtendMotor,1700,0.5); //1000
                follower.followPath(Line);
                setPathState(1);
                //Duce primul sample la cos (preload)
                break;

            case 1:
                if (!follower.isBusy() && !armLiftMotor.isBusy() && !armExtendMotor.isBusy()) {
                    MotorParameters(armLiftMotor, 30, 0.45);
                    Gheara.setPosition(0.88);
                    setPathState(2);
                    //Arunca Sample-ul in cos
                }
                break;

            case 2:
                if (!armLiftMotor.isBusy()) {
                    MotorParameters(armLiftMotor,-100,0.6);
                    setPathState(3);
                }
                break;
            case 3:
                if (!armLiftMotor.isBusy()) {
                    MotorParameters(armExtendMotor,100,0.5);
                    follower.followPath(Traj1, true);
                    setPathState(4);
                    //Case 2 + Case 3: duc la poziie normala sa nu pice bratul
                }
                break;

            case 4:
                if (!follower.isBusy() && !armExtendMotor.isBusy()) {
                    MotorParameters(armLiftMotor,1050,0.55);
                    MotorParameters(armExtendMotor, 300,0.5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    Gheara.setPosition(0.92);
                    GhearaTimp = System.currentTimeMillis();
                    setPathState(6);
                    //Case 4 + 5 --> Duce bratul la pozitie pentru prinderea primului element
                }
                break;
            case 6:
                if (System.currentTimeMillis() - GhearaTimp >= 550) {
                    MotorParameters(armLiftMotor,-45,0.55);
                    MotorParameters(armExtendMotor,1700,0.5);
                    setPathState(7);
                    //Extindere pentru cos - simultan cu miscarea pe traiectorie
                }
                break;

            case 7:
                follower.followPath(Sybau, true);
                setPathState(8);
                //Robotul merge pana la cos
                break;
            case 8:
                if (!follower.isBusy()) {
                    Gheara.setPosition(0.88);
                    setPathState(9);
                    //Da drop la al doilea sample
                }
                break;
            case 9:
                MotorParameters(armExtendMotor,100,0.5);
                follower.followPath(Traj2, true);
                setPathState(10);
                //Retrage glisiera si se duce la al treilea sample
                break;
            case 10:
                if (!follower.isBusy() && !armExtendMotor.isBusy()) {
                    MotorParameters(armLiftMotor,970,0.55);
                    MotorParameters(armExtendMotor, 400, 0.5);
                    setPathState(11);
                    //Duce bratul pentru a lua al treilea element
                }
                break;
            case 11:
                if (!armExtendMotor.isBusy() && !armLiftMotor.isBusy()) {
                    Gheara.setPosition(0.92);
                    GhearaTimp = System.currentTimeMillis();
                    setPathState(12);
                }
                break;
            case 12:
                if (System.currentTimeMillis() - GhearaTimp >= 550) {
                    MotorParameters(armLiftMotor,-45, 0.55);
                    MotorParameters(armExtendMotor,1700,0.5);
                    setPathState(13);
                    //Case 11 + 12 --> Prinde al treilea sample si duce bratul in pozitie de scoring
                }
                break;
            case 13:
                follower.followPath(Sybau2, true);
                setPathState(14);
                //Duce la cos

            case 14:
                if (!follower.isBusy()) {
                    Gheara.setPosition(0.88);
                    GhearaTimp = System.currentTimeMillis();
                    setPathState(15);
                    //Drop la sample
                }
                break;
            case 15:
                if(System.currentTimeMillis() - GhearaTimp >= 550) {
                    MotorParameters(armExtendMotor, 0, 0.5);
                    MotorParameters(armLiftMotor, 0, 0.55);
                    setPathState(16);
                    //Retragere la 0 pentru nu a avea probleme cu tickurile in TeleOP
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Park, true);
                    setPathState(17);
                    //Parcare
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    MotorParameters(ascentMotorLeft,200,0.45);
                    MotorParameters(ascentMotorRight,450,0.63);
                    //Ascent Level 1
                    setPathState(-1);
                }
                break;
        }
    }



    public void MotorParameters(DcMotor Motor, int Ticks, double Power){
        Motor.setTargetPosition(Ticks);
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor.setPower(Power);
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
        ascentMotorLeft = hardwareMap.get(DcMotor.class, "ascent_motor_left");
        ascentMotorRight = hardwareMap.get(DcMotor.class, "ascent_motor_right");
        ascentMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setDirection(DcMotor.Direction.REVERSE);


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

