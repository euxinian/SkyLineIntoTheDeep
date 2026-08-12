package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Movement Refactor", group="Iterative Opmode")
public class MovementRefactor extends OpMode
{
    //Hardware
    private DcMotor LF, RF, LR, RR  = null;
    private DcMotor armExtendMotor, armLiftMotor, ascentMotorLeft, ascentMotorRight = null;
    AnalogInput potentiometer;
    private Servo Gheara =  null;
    IMU imu;


    // PID
    public static double Kp = 0.00111, Ki = 0.00322, Kd = 0.00412;
    public static double integral = 0.0, lastError = 0.0;
    public static double KpAscent = 0.001, KiAscent = 0.002, KdAscent = 0.003;
    public static double integralAscent = 0.0, lastErrorAscent = 0.0, lastErrorAscent2 = 0.0;
    public static int TargetPositionPID = 0;


    //Potentiometru
    double minVoltage =0.435; // Tensiunea la pozitia minima
    double maxVoltage = 1.435;// Tensiunea la pozitia maxima
    double minAngle = 0.0;    // Unghiul la pozitia minima
    double maxAngle = 270.0;  // Unghiul la pozitia maxima
    double unghiBrat=0.0;

    //Automatizari
    boolean autolifeActive=false;
    public enum LiftState {
        VERTICAL,
        HORIZONTAL,
        EXTEND,
        RETRACT,
        SAMPLE,
        ASCENT,
        HOLD,
        PID
    }
    LiftState liftState = LiftState.HORIZONTAL;

    //INIT
    @Override
    public void init()
    {
        //Hardware Map
        LF = hardwareMap.get(DcMotor.class, "leftFront");
        RF = hardwareMap.get(DcMotor.class, "rightFront");
        LR = hardwareMap.get(DcMotor.class, "leftRear");
        RR = hardwareMap.get(DcMotor.class, "rightRear");
        Gheara = hardwareMap.get(Servo.class, "claw_servo");
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        armLiftMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        ascentMotorLeft = hardwareMap.get(DcMotor.class, "ascent_motor_left");
        ascentMotorRight = hardwareMap.get(DcMotor.class, "ascent_motor_right");

        //Directii
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);
        ascentMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        armExtendMotor.setDirection(DcMotor.Direction.REVERSE);

        //Brake
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascentMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascentMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Encoder
        ascentMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    //FUNCTIE CALCULARE UNGHI
    public double map(double value, double inMin, double inMax, double outMin, double outMax) {
        unghiBrat = (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return unghiBrat;
    }

    //FUNCTII PID + RESET PID
    private void maintainAscentPosition(double targetPosition, double targetPosition2) {
        int currentPositionLeft = ascentMotorLeft.getCurrentPosition();
        int currentPositionRight = ascentMotorRight.getCurrentPosition();
        double error =targetPosition - currentPositionRight;
        double error2 = targetPosition2 - currentPositionLeft;


        integralAscent = Range.clip(integralAscent + error + error2, -100.0, 100.0);
        double derivative = error - lastErrorAscent;
        double derivative2 = error2 - lastErrorAscent2;
        lastErrorAscent = error;
        lastErrorAscent2 = error2;

        double AscentpidOutput = KpAscent * error + KiAscent * integralAscent + KdAscent * derivative;
        double AscentpidOutput2 = KpAscent * error2 + KiAscent * integralAscent + KdAscent * derivative2;

        ascentMotorLeft.setPower(Range.clip(AscentpidOutput2, -0.02, 0.02));
        ascentMotorRight.setPower(Range.clip(AscentpidOutput, -0.032, 0.032));

        telemetry.addLine("┌──────────────────┐");
        telemetry.addData("│ ERR 1", "%.2f │", error);
        telemetry.addData("│ ERR 2", "%.2f │", error2);
        telemetry.addLine("├──────────────────┤");
        telemetry.addData("│ OUT 1", "%.4f │", AscentpidOutput);
        telemetry.addData("│ OUT 2", "%.4f │", AscentpidOutput2);
        telemetry.addLine("└──────────────────┘");
        telemetry.update();
    }

    private void maintainArmPosition(int x, int armExtendPos) {
        TargetPositionPID = armLiftMotor.getCurrentPosition();

        int armLiftPos = armLiftMotor.getCurrentPosition();
        double error = x - armLiftPos;

        integral = Range.clip(integral + error, -100.0, 100.0);

        double derivative = error - lastError;
        lastError = error;

        double extensionFactor = 1.0 + (armExtendPos / 3000.0);

        double pidOutput = (Kp * error + Ki * integral + Kd * derivative) * extensionFactor;

        armLiftMotor.setPower(Range.clip(pidOutput, -0.25, 0.25));
    }
    private void resetAscentPID() {
        integralAscent = 0.0;
        lastErrorAscent = 0.0;
        lastErrorAscent2 = 0.0;
    }
    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
    }
    public void MotorParameters(DcMotor Motor, int Ticks, double Power){
        Motor.setTargetPosition(Ticks);
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor.setPower(Power);
    }

    @Override
    public void loop(){
        //CAMERA (AM SCOS-O LMAO)

        //Valori potentiometru
        double voltage = potentiometer.getVoltage();
        unghiBrat = map(voltage, minVoltage, maxVoltage, minAngle, maxAngle);

        //Trigger
        double R2 = gamepad2.right_trigger *0.6;
        double L2 = gamepad2.left_trigger *0.6;

// 1) Mecanum Drive/Movement

        /*double y = (-gamepad1.left_stick_y)*0.8;
        double x = (gamepad1.left_stick_x * 1.1)*0.8;
        double rx = gamepad1.right_stick_x *0.8;

        double dominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) ,1);
        double LFpower = (y + x + rx) / dominator;
        double LRpower = (y - x + rx) / dominator;
        double RFpower = (y - x - rx) / dominator;
        double RRpower = (y + x - rx) / dominator;

        LF.setPower(LFpower);
        LR.setPower(LRpower);
        RF.setPower(RFpower);
        RR.setPower(RRpower);*/

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);

// 2) Pozitii Gheara

        if (gamepad1.right_bumper) Gheara.setPosition(0.85);
        else if(gamepad1.left_bumper) Gheara.setPosition(0.9);

// 3) Brat/Extend/Ascent/Auomatizari

        int armExtendPos = armExtendMotor.getCurrentPosition();
        int TargetPositionPID = armLiftMotor.getCurrentPosition();
        double RightStick=-gamepad2.right_stick_y;

        if(R2 > 0.1 || L2>0.1){
            autolifeActive = false;
            armExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(R2 > 0.05 ) armExtendMotor.setPower(R2*0.8);
            else if(L2 > 0.05 && armExtendPos>100) armExtendMotor.setPower(-L2*0.8);

        } else if(R2<0.1 && L2<0.1 && !autolifeActive) armExtendMotor.setPower(0.0002);

        if(gamepad2.x){
            liftState = LiftState.VERTICAL;
            autolifeActive = true;
        }
        if(gamepad2.y){
            liftState = LiftState.RETRACT;
            autolifeActive = true;
        }
        if(gamepad1.y){
            liftState = LiftState.ASCENT;
            autolifeActive = true;
        }
        /*if(gamepad2.right_bumper){
            liftState = LiftState.SAMPLE;
            autolifeActive = true;
        } - Deprecated
        */

        if(autolifeActive){
            switch(liftState){
                case VERTICAL:
                    MotorParameters(armLiftMotor,-50, -0.5);
                    if(!armLiftMotor.isBusy()){
                        liftState= LiftState.EXTEND;
                    }
                    break;
                case HORIZONTAL:
                    MotorParameters(armLiftMotor,516,0.5);
                    if(!armLiftMotor.isBusy()){
                        autolifeActive = false;
                    }
                    break;
                case EXTEND:
                    MotorParameters(armExtendMotor,1700,0.6);
                    if(!armExtendMotor.isBusy()){
                        autolifeActive = false;
                        break;
                    }
                    break;
                case RETRACT:
                    MotorParameters(armExtendMotor,50,-0.6);
                    if(!armExtendMotor.isBusy()){
                        liftState = LiftState.HORIZONTAL;
                    }
                    break;
                case SAMPLE:
                    MotorParameters(armLiftMotor,965,0.4);
                    if(!armLiftMotor.isBusy()){
                        autolifeActive = false;
                    }
                    break;
                case ASCENT:
                    MotorParameters(ascentMotorLeft,3300,0.45);
                    MotorParameters(ascentMotorRight,4620,0.63);
                    if(!ascentMotorLeft.isBusy() && !ascentMotorRight.isBusy()){
                        liftState = LiftState.HOLD;
                    }
                    break;
                case HOLD:
                    MotorParameters(ascentMotorLeft,1500,-0.45);
                    MotorParameters(ascentMotorRight,2000,-0.63);
                    if(!ascentMotorLeft.isBusy() && !ascentMotorRight.isBusy()){
                        liftState=LiftState.PID;
                    }
                    break;
                case PID:
                    maintainAscentPosition(2000, 1500);
                    resetAscentPID();
                    if(!ascentMotorLeft.isBusy() && !ascentMotorRight.isBusy()){
                        autolifeActive=false;
                    }
                    break;
            }
        }

        if(Math.abs(RightStick) > 0.1){
            autolifeActive=false;
            armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(unghiBrat<22 && RightStick>0) armLiftMotor.setPower(0);
            else if(unghiBrat>245 && RightStick<0) armLiftMotor.setPower(0);
            else armLiftMotor.setPower(RightStick*0.4);

            resetPID();
        } else if(Math.abs(RightStick) < 0.1 && !autolifeActive) {
            maintainArmPosition(TargetPositionPID, armExtendPos);
        }

// 4) Telemetry
        telemetry.addLine("╔══════════════════╗");
        telemetry.addData("║ ASCENT TICKS", " L: %d | R: %d",
                ascentMotorLeft.getCurrentPosition(),
                ascentMotorRight.getCurrentPosition());
        telemetry.addData("║ ASCENT PWR", " L: %.2f | R: %.2f",
                ascentMotorLeft.getPower(),
                ascentMotorRight.getPower());
        telemetry.addData("║ ARM TICKS", " Lift: %d | Ext: %d",
                armLiftMotor.getCurrentPosition(),
                armExtendMotor.getCurrentPosition());
        telemetry.addLine("╚══════════════════╝");
        telemetry.update();
    }
}

