package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CarouselSpinner {

    private CRServo carouselSpinnerServo;

    public CarouselSpinner(CRServo carouselSpinnerServo){
        this.carouselSpinnerServo = carouselSpinnerServo;
    }

    public void spin(boolean reverse){
        if (reverse){
            carouselSpinnerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        carouselSpinnerServo.setPower(1.0);
    }

    public void stop(){
        carouselSpinnerServo.setPower(0.0);
    }
}
