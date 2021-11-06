package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {

    CarouselSpinner carouselSpinner;
    CRServo carouselSpinnerServo;

    Lift lift;
    DcMotor liftMotor;

    Arm arm;
    DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerServo = (CRServo)hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerServo);

        liftMotor = (DcMotor)hardwareMap.get("liftMotor");
        lift = new Lift(liftMotor);

        armMotor = (DcMotor)hardwareMap.get("armMotor");
        arm = new Arm(armMotor);

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

            if (gamepad1.left_trigger > 0){
                lift.up(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0){
                lift.down(gamepad1.right_trigger);
            }

            if (gamepad2.left_trigger > 0){
                arm.rotate(gamepad2.left_trigger, true);
            }
            else if (gamepad2.right_trigger > 0){
                arm.rotate(gamepad2.right_trigger, false);
            }
            else {
                arm.rotate(0.0, true);
            }
        }
    }
}
