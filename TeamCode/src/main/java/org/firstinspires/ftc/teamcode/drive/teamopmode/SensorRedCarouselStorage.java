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
public class SensorRedCarouselStorage extends LinearOpMode {

    private CarouselSpinner carouselSpinner = null;
    private DcMotor carouselSpinnerMotor = null;

    private Arm arm = null;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    private ShippingElementDetector shippingElementDetector = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;

    private Pose2d poseHome = new Pose2d(0, 0, 0);
    private Pose2d poseCarousel = new Pose2d(-15,5.355,0);
    private Pose2d poseDetectRandomization = new Pose2d(0.743,20.186,1.571);
    private Pose2d poseShippingHub1 = new Pose2d(21.712, 9.811, 1.427);
    private Pose2d poseShippingHub2 = new Pose2d(23.924, 22.819, 1.427);
    private Pose2d poseStorage = new Pose2d(-21.066, 27.936, 0);

    private Trajectory trajectoryHomeToCarouselSpinner = null;
    private Trajectory trajectoryCarouselSpinnerToDetect = null;
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
        trajectoryHomeToCarouselSpinner = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseCarousel)
                .build();

        drive.followTrajectory(trajectoryHomeToCarouselSpinner);
    }

    private void spinCarousel(CarouselSpinner carouselSpinner){
        carouselSpinner.spin(true, 0.5);
        this.sleep(2500);
        carouselSpinner.stop();
    }

    private void driveToDetect(SampleMecanumDrive drive){
        trajectoryCarouselSpinnerToDetect = drive.trajectoryBuilder(trajectoryHomeToCarouselSpinner.end())
                .lineToLinearHeading(poseDetectRandomization)
                .build();

        drive.followTrajectory(trajectoryCarouselSpinnerToDetect);
    }

    private double detectShippingElementAndReturnYOffset(ShippingElementDetector shippingElementDetector){
        String location = shippingElementDetector.detectPringle();
        double yOffset = 0;

        if (ShippingElementDetector.NEITHER.equals(location)){
            armPosition = Arm.HIGH_POSITION;
            yOffset = 2;
        }
        else if (ShippingElementDetector.LEFT.equals(location)){
            armPosition = Arm.LOW_POSITION;
        }

        return yOffset;
    }

    private void driveToShippingHub(SampleMecanumDrive drive, Arm arm, double yOffset){
        poseShippingHub2 = poseShippingHub2.plus(new Pose2d(0, yOffset, 0));

        trajectoryDetectToShippingHub1 = drive.trajectoryBuilder(trajectoryCarouselSpinnerToDetect.end())
                .lineToLinearHeading(poseShippingHub1,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
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