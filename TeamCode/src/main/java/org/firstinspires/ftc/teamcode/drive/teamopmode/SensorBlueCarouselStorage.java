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

    private CarouselSpinner carouselSpinner = null;
    private DcMotor carouselSpinnerMotor = null;

    private Arm arm = null;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    private ShippingElementDetector shippingElementDetector = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;

    private Pose2d poseHome = new Pose2d(0, 0, 0);
    private Pose2d poseCarousel1 = new Pose2d(0,-3.5,0);
    private Pose2d poseCarousel2 = new Pose2d(-5.1, -3.5, 5.6);
    private Pose2d poseAvoidDuck = new Pose2d(-4.1, -8.5, 5.6);
    private Pose2d poseDetectRandomization = new Pose2d(12.75,-19.25,4.712);
    private Pose2d poseShippingHub1 = new Pose2d(36.049, -3.5, 5.05);
    private Pose2d poseShippingHub2 = new Pose2d(36.049, -19.385, 5.05);
    private Pose2d poseStorage = new Pose2d(-9.970, -27.670, 0);

    private Trajectory trajectoryHomeToCarouselSpinner1 = null;
    private Trajectory trajectoryCarouselSpinner1ToCarouselSpinner2 = null;
    private Trajectory trajectoryCarouselSpinner2ToAvoidDuck = null;
    private Trajectory trajectoryAvoidDuckToDetect = null;
    private Trajectory trajectoryDetectToShippingHub1 = null;
    private Trajectory trajectoryShippingHub1ToShippingHub2 = null;
    private Trajectory trajectoryShippingHub2ToShippingHub1 = null;
    private Trajectory trajectoryShippingHub1ToStorage = null;

    private int armPosition = Arm.MID_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);

        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        shippingElementDetector = new ShippingElementDetector(leftSensor, rightSensor);

        waitForStart();

        runAutonomous(drive, carouselSpinner, arm, shippingElementDetector);
    }

    private void runAutonomous(SampleMecanumDrive drive, CarouselSpinner carouselSpinner, Arm arm, ShippingElementDetector shippingElementDetector) {
        arm.grab();
        sleep(1500);
        arm.move(Arm.MID_POSITION);
        driveToCarousel(drive);
        spinCarousel(carouselSpinner);
        driveToDetect(drive);
        double yOffset = detectShippingElementAndReturnYOffset(shippingElementDetector);
        driveToShippingHub(drive, arm, yOffset);
        arm.release();
        sleep(1000);
        driveToPark(drive);
        arm.move(Arm.PARK_POSITION);
    }

    private void driveToCarousel(SampleMecanumDrive drive){
        trajectoryHomeToCarouselSpinner1 = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseCarousel1)
                .build();

        trajectoryCarouselSpinner1ToCarouselSpinner2 = drive.trajectoryBuilder(trajectoryHomeToCarouselSpinner1.end())
                .lineToLinearHeading(poseCarousel2,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        drive.followTrajectory(trajectoryHomeToCarouselSpinner1);
        drive.followTrajectory(trajectoryCarouselSpinner1ToCarouselSpinner2);
    }

    private void spinCarousel(CarouselSpinner carouselSpinner){
        carouselSpinner.spin(false, 0.4);
        this.sleep(3000);
        carouselSpinner.stop();
    }

    private void driveToDetect(SampleMecanumDrive drive){
        trajectoryCarouselSpinner2ToAvoidDuck = drive.trajectoryBuilder(trajectoryCarouselSpinner1ToCarouselSpinner2.end())
                .lineToLinearHeading(poseAvoidDuck)
                .build();

        trajectoryAvoidDuckToDetect = drive.trajectoryBuilder(trajectoryCarouselSpinner2ToAvoidDuck.end())
                .lineToLinearHeading(poseDetectRandomization)
                .build();

        drive.followTrajectory(trajectoryCarouselSpinner2ToAvoidDuck);
        drive.followTrajectory(trajectoryAvoidDuckToDetect);
    }

    private double detectShippingElementAndReturnYOffset(ShippingElementDetector shippingElementDetector){
        String location = shippingElementDetector.detectPringle();
        double yOffset = 0;

        if (ShippingElementDetector.RIGHT.equals(location)){
            armPosition = Arm.HIGH_POSITION;
            yOffset = -4;
        }
        else if (ShippingElementDetector.NEITHER.equals(location)){
            armPosition = Arm.LOW_POSITION;
            yOffset = -3;
        }

        return yOffset;
    }

    private void driveToShippingHub(SampleMecanumDrive drive, Arm arm, double yOffset){
        poseShippingHub2 = poseShippingHub2.plus(new Pose2d(0, yOffset, 0));

        trajectoryDetectToShippingHub1 = drive.trajectoryBuilder(trajectoryAvoidDuckToDetect.end())
                .lineToLinearHeading(poseShippingHub1)
                .build();

        trajectoryShippingHub1ToShippingHub2 = drive.trajectoryBuilder(trajectoryDetectToShippingHub1.end())
                .lineToLinearHeading(poseShippingHub2)
                .build();

        drive.followTrajectory(trajectoryDetectToShippingHub1);

        arm.move(armPosition);

        drive.followTrajectory(trajectoryShippingHub1ToShippingHub2);
    }

    private void driveToPark(SampleMecanumDrive drive){
        trajectoryShippingHub2ToShippingHub1 = drive.trajectoryBuilder(trajectoryShippingHub1ToShippingHub2.end())
                .lineToLinearHeading(poseShippingHub1)
                .build();

        trajectoryShippingHub1ToStorage = drive.trajectoryBuilder(trajectoryShippingHub2ToShippingHub1.end())
                .lineToLinearHeading(poseStorage)
                .build();

        drive.followTrajectory(trajectoryShippingHub2ToShippingHub1);
        drive.followTrajectory(trajectoryShippingHub1ToStorage);
    }
}
