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


@Autonomous(name = "Autonomie Bara", group = "Examples")
public class AutoBara extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor armExtendMotor, armLiftMotor, ascentMotorLeft, ascentMotorRight = null;
    private Servo Gheara;

    private long GhearaTimp; //Masurarea timpului pentru a lasa gheara sa prinda elemente de joc
    //Masuram momentul in care gheara este inchisa, dupa care robotul se misca DUPA 500ms



    //-------------------------------------------------------------------------------------\\
    private final Pose StartPositon = new Pose(1.32, 95.92, Math.toRadians(90)); //1.32, 95.92
    private final Pose Chamber = new Pose(28.14, 87.28, Math.toRadians(0)); //24.14, 31.26 --- 24.14, 87.28
    private final Pose Sample1 = new Pose(2.21, 31.45, Math.toRadians(-90));
    private final Pose Control1 = new Pose(0.166, 42.73, Math.toRadians(180));
    private final Pose Cotnrol2 = new Pose(110.74, 29.09, Math.toRadians(180));
    private final Pose Sample2 = new Pose(3.54, 25.03, Math.toRadians(180));
    private final Pose Control3 = new Pose(89.62, 50.05, Math.toRadians(180));
    private final Pose Control4 = new Pose(74.32, 20.61, Math.toRadians(180));
    private final Pose Pick = new Pose(2.99, 44.72, Math.toRadians(180));
    private final Pose Chamber2 = new Pose(29.67, 79.93, Math.toRadians(0)); //33.09, 80.97  --? 24.11, 80.14
    private final Pose Control5 = new Pose(11.63, 64.18, Math.toRadians(0));
    private final Pose Chamber3 = new Pose(29.67, 73.17, Math.toRadians(0)); //32.75, 76.48




    //-----------------------------------------------------------------------------\\
    private Path Spike1, Spike2, Score1, Pickup2, Score2 ;
    private PathChain Preload, Pickup1;
    //------------------------------------------------------------------------------\\




    public void buildPaths(){
        Preload=follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartPositon), new Point(Chamber)))
                .setLinearHeadingInterpolation(StartPositon.getHeading(), Chamber.getHeading())
                .build();

        Spike1 = new Path(new BezierCurve(new Point(Chamber), new Point(Control1), new Point(Cotnrol2), new Point(Sample1)));
        Spike1.setLinearHeadingInterpolation(Chamber.getHeading(), Sample1.getHeading());

        Spike2 = new Path(new BezierCurve(new Point(Sample1), new Point(Control3),new Point(Control4), new Point(Sample2)));
        Spike2.setLinearHeadingInterpolation(Sample1.getHeading(), Sample2.getHeading());

        Pickup1=follower.pathBuilder()
                .addPath(new BezierLine(new Point(Sample2), new Point(Pick)))
                .setLinearHeadingInterpolation(Sample2.getHeading(), Pick.getHeading())
                .build();

        Score1 = new Path(new BezierCurve(new Point(Pick), new Point(Control5), new Point(Chamber2)));
        Score1.setLinearHeadingInterpolation(Pick.getHeading(), Chamber2.getHeading());

        Pickup2 = new Path(new BezierCurve(new Point(Chamber2), new Point(Control5), new Point(Pick)));
        Pickup2.setLinearHeadingInterpolation(Chamber2.getHeading(), Pick.getHeading());

        Score2 = new Path(new BezierCurve(new Point(Pick), new Point(Control5), new Point(Chamber3)));
        Score2.setLinearHeadingInterpolation(Pick.getHeading(), Chamber3.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Gheara.setPosition(0.92);
                follower.followPath(Preload);
                MotorParameters(armLiftMotor, 80, 0.6);
                setPathState(1);
                //Duce preloadul la bara si il impinge in ea
                break;
            case 1:
                if(!follower.isBusy() && !armLiftMotor.isBusy()){
                    MotorParameters(armExtendMotor, 920, 0.6);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && !armExtendMotor.isBusy()){
                    Gheara.setPosition(0.89);
                    MotorParameters(armExtendMotor, 0, 0.8);
                    follower.followPath(Spike1);
                    //Da drumul la gheara, retrage bratul si pozitie de luare element de la observation zone
                    //Merge sa duca primul spike in observation zone
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    MotorParameters(armLiftMotor, 800, 0.6);
                    follower.followPath(Spike2);
                    //Duce al doilea spike in observation zone
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Pickup1);
                    //Merge la pozitia de luat element
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    MotorParameters(armExtendMotor, 500,0.5);
                    setPathState(6);
                }
                break;
            case 6:
                if(!armExtendMotor.isBusy()){
                    Gheara.setPosition(0.92);
                    GhearaTimp=System.currentTimeMillis();
                    setPathState(7);
                }
                break;
            case 7:
                if(System.currentTimeMillis() - GhearaTimp >=500){
                    follower.followPath(Score1);
                    MotorParameters(armLiftMotor, 0, 0.6); //-20
                    //MotorParameters(armExtendMotor, 100, 0.7);
                    setPathState(8);
                    // 4 - se extinde bratul
                    // 5 - Ia elementul
                    // 6 - Dupa ce ia elementul si il prinde, merge la pozitii arbitrare pentru bara
                    // Astfel incat sa nu incurce robotul din alianta, apoi urmareste traiectoria
                }
                break;
            case 8:
                if(!follower.isBusy() && !armLiftMotor.isBusy()){
                    MotorParameters(armExtendMotor, 915, 0.6);
                    setPathState(9);
                    //Impinge elementul pe chamber
                }
                break;
            case 9:
                if(!armExtendMotor.isBusy()){
                    Gheara.setPosition(0.88);
                    GhearaTimp=System.currentTimeMillis();
                    setPathState(10);
                }
                break;
            case 10:
                if(System.currentTimeMillis() - GhearaTimp >= 700){
                    MotorParameters(armExtendMotor, 500, 0.5);
                    setPathState(11);
                    // 9 - Deschide gheara
                    // 10 - Ca sa nu se blocheze pe bara, dupa 300ms bratul (Lift + Extend) revine la pozitie normala
                    //!! Testeaza, daca nu pune if(!armLiftMotor.isBusy()) MotorParameters(armExtendMotor, 0, 0.5);
                }
                break;
            case 11:
                if(!armExtendMotor.isBusy()) {
                    follower.followPath(Pickup2);
                    MotorParameters(armLiftMotor,800,0.5);
                    setPathState(12);
                }
                //Se opreste autonomia
                //Verifica timpul si vezi daca mai putem pune un element sau facem parcare
                break;
            case 12:
                if(!follower.isBusy() && !armLiftMotor.isBusy()) {
                    Gheara.setPosition(0.92);
                    GhearaTimp = System.currentTimeMillis();
                    setPathState(13);
                }

                break;
            case 13:
                if(System.currentTimeMillis() - GhearaTimp >= 1000 ) {
                    MotorParameters(armLiftMotor, 31, 0.6); //-20
                    follower.followPath(Score2);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy() && !armLiftMotor.isBusy()) {
                    MotorParameters(armExtendMotor, 915, 0.6);
                    setPathState(15);
                }
                break;
            case 15:
                if(!armExtendMotor.isBusy()) {
                    Gheara.setPosition(0.88);
                    MotorParameters(armExtendMotor, 0, 0.6);
                    MotorParameters(armLiftMotor, 20, 0.6);
                    setPathState(16);
                }
                break;
            case 16:
               setPathState(-1);
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
        //ascentMotorLeft.setDirection(DcMotor.Direction.REVERSE);
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

