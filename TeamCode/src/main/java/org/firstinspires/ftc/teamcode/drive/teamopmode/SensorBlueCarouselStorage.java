package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorBlueCarouselStorage extends LinearOpMode {

    CarouselSpinner carouselSpinner = null;
    DcMotor carouselSpinnerMotor = null;

    Arm arm = null;
    DcMotor armMotor = null;
    Servo handServo = null;

    ShippingElementDetector pringles = null;
    DistanceSensor leftSensor = null;
    DistanceSensor rightSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo, true);
        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        pringles = new ShippingElementDetector(leftSensor, rightSensor);

        Trajectory toCarouselSpinner = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d( 0,-3.5))
                .build();

        Trajectory toCarouselSpinner2 = drive.trajectoryBuilder(toCarouselSpinner.end())
                .lineToLinearHeading(new Pose2d(-5.1, -3.5, 5.6),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory avoidDuck = drive.trajectoryBuilder(toCarouselSpinner2.end())
                .lineToLinearHeading(new Pose2d(-4.1, -8.5, 5.6))
                .build();

        Trajectory toDetect = drive.trajectoryBuilder(avoidDuck.end())
                .lineToLinearHeading(new Pose2d(14.834, -18.752, 4.712))
                .build();

        Trajectory toShippingMid = drive.trajectoryBuilder(toDetect.end())
                .lineToLinearHeading(new Pose2d(36.049, -3.5, 5.05))
                .build();

        waitForStart();

        arm.grab();
        this.sleep(1500);
        arm.move(Arm.MID_POSITION);
        this.sleep(500);
        drive.followTrajectory(toCarouselSpinner);
        this.sleep(500);
        drive.followTrajectory(toCarouselSpinner2);
        carouselSpinner.spin(false, 0.5);
        this.sleep(2500);
        carouselSpinner.stop();
        drive.followTrajectory(avoidDuck);
        drive.followTrajectory(toDetect);
        String location = pringles.detectPringle();
        double yOffset = 0;
        if (location.equals(ShippingElementDetector.RIGHT)){
            arm.move(Arm.HIGH_POSITION);
            yOffset = -2;
        }else if (location.equals(ShippingElementDetector.NEITHER)){
            arm.move(Arm.LOW_POSITION);
            yOffset = -3;
        }

        Trajectory toShippingMid2 = drive.trajectoryBuilder(toShippingMid.end())
                .lineToLinearHeading(new Pose2d(36.049, -19.385 + yOffset, 5.05))
                .build();

        Trajectory toPark = drive.trajectoryBuilder(toShippingMid2.end())
                .lineToLinearHeading(new Pose2d(36.049, -3.5, 5.05))
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark.end())
                .lineToLinearHeading(new Pose2d(-9.970, -27.670, 0))
                .build();

        this.sleep(500);
        drive.followTrajectory(toShippingMid);
        this.sleep(500);
        drive.followTrajectory(toShippingMid2);
        arm.release();
        this.sleep(1500);
        drive.followTrajectory(toPark);
        this.sleep(500);
        drive.followTrajectory(toPark2);
        arm.move(Arm.PARK_POSITION);
    }
}