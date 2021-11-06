package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {

    private static final double MAX_ROTATION_SPEED = 0.25;
    DcMotor armMotor = null;

    public Arm(DcMotor armMotor){
        this.armMotor = armMotor;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
