package org.firstinspires.ftc.teamcode.drive.teamopmode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class BlueTeleOp extends RedTeleOp{

    protected void spinCarousel(){
        carouselSpinner.spin(true);
    }
}
