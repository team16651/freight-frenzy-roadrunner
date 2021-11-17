package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class RedTeleOp extends LinearOpMode {

    CarouselSpinner carouselSpinner;
    DcMotor carouselSpinnerMotor;

    Lift lift;
    DcMotor liftMotor;

    Arm arm;
    DcMotor armMotor;
    Servo handServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);

        liftMotor = (DcMotor)hardwareMap.get("liftMotor");
        lift = new Lift(liftMotor);

        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            /* Controller 1 */
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.right_trigger > 0){
                spinCarousel(gamepad1.right_trigger);
            }
            else{
                carouselSpinner.stop();
            }

            /* Controller 2 */

            if (gamepad2.left_trigger > 0){
                arm.rotate(gamepad2.left_trigger, true);
            }
            else if (gamepad2.right_trigger > 0){
                arm.rotate(gamepad2.right_trigger, false);
            }
            else {
                arm.rotate(0.0, true);
            }

            if (gamepad2.left_stick_y > 0){
                lift.up(gamepad1.left_stick_y);
            }
            else if (gamepad2.left_stick_y < 0){
                lift.down(-gamepad1.left_stick_y);
            }

            if (gamepad2.y){
                arm.release();
            }

            if (gamepad2.a){
                arm.grab();
            }

            telemetry.addData("servo position", handServo.getPosition());
            telemetry.update();
        }
    }

    protected void spinCarousel(double speed){
        carouselSpinner.spin(true, speed);
    }
}

