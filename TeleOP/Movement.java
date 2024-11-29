package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOP", group="Iterative Opmode")
public class Movement extends OpMode
{
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LR = null;
    private DcMotor RR = null;
    private Servo Gheara = null;
    private DcMotor armExtendMotor = null;

    @Override
    public void init(){

        LF = hardwareMap.get(DcMotor.class, "left_front");
        RF = hardwareMap.get(DcMotor.class, "right_front");
        LR = hardwareMap.get(DcMotor.class, "left_rear");
        RR = hardwareMap.get(DcMotor.class, "right_rear");
        Gheara = hardwareMap.get(Servo.class, "claw_servo");
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_extend_motor");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gheara.setPosition(0);
    }

    @Override
    public void loop(){

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        boolean button_a = gamepad1.a;
        boolean button_b = gamepad1.b;

        double rotire = gamepad1.right_stick_x;
        double alfa = Math.atan2(y, x);
        double pow = Math.hypot(y, x);

        double sin = Math.sin(alfa - Math.PI/4);
        double cos = Math.cos(alfa - Math.PI/4);
        double max;

        if(Math.abs(y)<0.05 && Math.abs(x)<0.05 && Math.abs(-rotire)<0.05){
            LF.setPower(0);
            RF.setPower(0);
            LR.setPower(0);
            RR.setPower(0);
        } else{
            double PUTERE_LF = y + x + rotire;
            double PUTERE_RF = y - x - rotire;
            double PUTERE_LR = y - x + rotire;
            double PUTERE_RR = y + x - rotire;

            max = Math.max(Math.abs(PUTERE_LF), Math.abs(PUTERE_RF));
            max = Math.max(max, Math.abs(PUTERE_LR));
            max = Math.max(max, Math.abs(PUTERE_RR));

            if(max > 1.0){
                PUTERE_LF /= max;
                PUTERE_RF /= max;
                PUTERE_LR /= max;
                PUTERE_RR /= max;
            }

            LF.setPower(PUTERE_LF);
            RF.setPower(PUTERE_RF);
            LR.setPower(PUTERE_LR);
            RR.setPower(PUTERE_RR);
        }

        if(button_a){
            Gheara.setPosition(0.16);
        }
        if(button_b){
            Gheara.setPosition(0.115);
        }

        int armPos=armExtendMotor.getCurrentPosition();
        double armPower = -gamepad2.right_stick_y/2;
        if (armPower>0 && armPos<1000)
            armExtendMotor.setPower(armPower);
        else if (armPower<0 && armPos>2)
            armExtendMotor.setPower(armPower);
        else{
            armPower=0;
        }




    }
}


// Discord @euxinian
// Bagati si voi discordurile :)))
