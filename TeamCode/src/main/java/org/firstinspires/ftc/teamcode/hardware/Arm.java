package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {

    private static final double MAX_ROTATION_SPEED = 0.50;
    DcMotor armMotor = null;
    Servo handServo = null;

    public Arm(DcMotor armMotor, Servo handServo){
        this.armMotor = armMotor;
        this.handServo = handServo;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void grab(){
        handServo.setPosition(0);
    }

    public void release(){
        handServo.setPosition(0.35);
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
