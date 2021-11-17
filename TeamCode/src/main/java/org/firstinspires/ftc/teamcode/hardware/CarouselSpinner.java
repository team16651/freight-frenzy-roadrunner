package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CarouselSpinner {

    private DcMotor carouselSpinnerMotor;

    public CarouselSpinner(DcMotor carouselSpinnerMotor){
        this.carouselSpinnerMotor = carouselSpinnerMotor;
        carouselSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(boolean reverse, double speed){
        if (reverse){
            carouselSpinnerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            carouselSpinnerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        carouselSpinnerMotor.setPower(speed);
    }

    public void stop(){
        carouselSpinnerMotor.setPower(0.0);
    }
}
