package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    DcMotor liftMotor = null;

    public Lift(DcMotor liftMotor){
        this.liftMotor = liftMotor;
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up(double power){
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setPower(power);
    }

    public void down(double power){
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(power);
    }

}
