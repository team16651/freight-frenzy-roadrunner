package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private static final double MAX_ROTATION_SPEED = 1.0;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    public static final int GROUND = 0;
    public static final int PARK_POSITION = 100;
    public static final int LOW_POSITION = 370;
    public static final int MID_POSITION = 840;
    public static final int HIGH_POSITION = 1270;

    private int currentState = GROUND;

    public Arm(DcMotor armMotor, Servo handServo){
        this.armMotor = armMotor;
        this.handServo = handServo;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Arm(DcMotor armMotor, Servo handServo, boolean autonomous){
        this(armMotor, handServo);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void grab(){
        handServo.setDirection(Servo.Direction.FORWARD);
        handServo.setPosition(0.0);
    }

    public void release(){
        handServo.setDirection(Servo.Direction.REVERSE);
        handServo.setPosition(0.5);
    }

    public int move (int targetState){
        armMotor.setTargetPosition(targetState);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetState != GROUND) {
            armMotor.setPower(0.25);
        }
        else{
            armMotor.setPower(0.0);
        }

        return armMotor.getCurrentPosition();
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