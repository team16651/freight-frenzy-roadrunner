package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorBlueWarehouseOutward extends LinearOpMode {

    Arm arm = null;
    DcMotor armMotor = null;
    Servo handServo = null;

    ShippingElementDetector pringles = null;
    DistanceSensor leftSensor = null;
    DistanceSensor rightSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo, true);
        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        pringles = new ShippingElementDetector(leftSensor, rightSensor);

        Trajectory toDetect = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(13.076,-19.959,4.712))
                .build();

        waitForStart();
        arm.grab();
        this.sleep(1500);
        arm.move(Arm.MID_POSITION);
        this.sleep(500);
        drive.followTrajectory(toDetect);
        this.sleep(500);

        String location = pringles.detectPringle();
        double yOffset = 0;
        if (location.equals(ShippingElementDetector.RIGHT)){
            arm.move(Arm.HIGH_POSITION);
            yOffset = -2;
        }else if (location.equals(ShippingElementDetector.NEITHER)){
            arm.move(Arm.LOW_POSITION);
            yOffset = -1;
        }

        Trajectory toShippingHub = drive.trajectoryBuilder(toDetect.end())
                .lineToLinearHeading(new Pose2d(-2.105, -21.214 + yOffset, 4.354))
                .build();

        Trajectory toHome = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(0,0,0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory toWarehouse = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(44.698, -1.62))
                .build();
        this.sleep(1500);
        drive.followTrajectory(toShippingHub);
        arm.release();
        this.sleep(1500);
        drive.followTrajectory(toHome);
        this.sleep(500);
        drive.followTrajectory(toWarehouse);
        arm.move(Arm.PARK_POSITION);
    }
}