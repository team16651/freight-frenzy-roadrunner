package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {

    private static final double MAX_ROTATION_SPEED = 0.50;
    DcMotor armMotor = null;
    Servo handServo = null;

    public static final int GROUND = 0;
    public static final int LOW_POSITION = 51;
    public static final int MID_POSITION = 105;
    public static final int HIGH_POSITION = 172;


    private int currentState = GROUND;

    public Arm(DcMotor armMotor, Servo handServo){
        this.armMotor = armMotor;
        this.handServo = handServo;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void grab(){
        handServo.setDirection(Servo.Direction.FORWARD);
        handServo.setPosition(0.35);
    }

    public void release(){
        handServo.setDirection(Servo.Direction.REVERSE);
        handServo.setPosition(0);
    }

    public int calculateTargetPosition(boolean up, int currentState){
        return 0;
    }

    public void togglePosition(boolean up){
        int targetState = GROUND;
        int targetNum = 0;


        if (up){
            if (targetNum == 0){
                targetState = LOW_POSITION;
                targetNum = 1;
            }
            else if (targetNum == 1){
                targetState = MID_POSITION;
                targetNum = 2;
            }
        }
        else{
            if (targetNum == 1){
                targetState = GROUND;
                targetNum = 0;
            }else if (targetNum == 2){
                targetState = LOW_POSITION;
                targetNum = 1;
            }
        }
        currentState = targetState;

    }


    public void move (int targetState){
        armMotor.setTargetPosition(targetState);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetState != GROUND) {
            armMotor.setPower(1.0);
        }
        else{
            armMotor.setPower(0.0);
        }

    }
    public void rotate(double power, boolean up){
        if (up) {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else{
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        armMotor.setPower(power * MAX_ROTATION_SPEED);
    }

}
