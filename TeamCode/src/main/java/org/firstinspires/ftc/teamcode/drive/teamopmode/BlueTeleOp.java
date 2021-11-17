package org.firstinspires.ftc.teamcode.drive.teamopmode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class BlueTeleOp extends RedTeleOp{

    protected void spinCarousel(double speed){
        carouselSpinner.spin(false, speed);
    }
}
