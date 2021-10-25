package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {

    CarouselSpinner carouselSpinner;
    CRServo carouselSpinnerServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerServo = (CRServo)hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerServo);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.a){
                carouselSpinner.spin(false);
            }
            else if (gamepad1.b){
                carouselSpinner.spin(true);
            }
            else{
                carouselSpinner.stop();
            }
        }
    }
}
