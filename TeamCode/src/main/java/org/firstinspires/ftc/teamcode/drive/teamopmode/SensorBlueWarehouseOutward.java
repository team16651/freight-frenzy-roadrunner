package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorBlueWarehouseOutward extends LinearOpMode {

    private Arm arm = null;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    private ShippingElementDetector shippingElementDetector = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;

    private Pose2d poseHome = new Pose2d(0, 0, 0);
    private Pose2d poseDetectRandomization = new Pose2d(13.076,-19.959,4.712);
    private Pose2d poseShippingHub = new Pose2d(-2.105, -21.214, 4.354);
    private Pose2d poseWarehousePose = new Pose2d(44.698, -1.62, 0);

    protected Trajectory trajectoryHomeToDetect = null;
    protected Trajectory trajectoryDetectToShippingHub = null;
    protected Trajectory trajectoryShippingHubToHome = null;
    protected Trajectory trajectoryHomeToWarehouse = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        shippingElementDetector = new ShippingElementDetector(leftSensor, rightSensor);

        waitForStart();

        runAutonomous(drive, arm, shippingElementDetector);
    }

    protected void runAutonomous(SampleMecanumDrive drive, Arm arm, ShippingElementDetector shippingElementDetector){
        arm.grab();
        sleep(1500);
        arm.move(Arm.MID_POSITION);
        driveToDetect(drive);
        double yOffset = detectShippingElementAndReturnYOffset(shippingElementDetector, arm);
        driveToShippingHub(drive, yOffset);
        arm.release();
        sleep(1000);
        driveToHome(drive);
        driveToWarehouse(drive);
        arm.move(Arm.PARK_POSITION);
    }

    private void driveToDetect(SampleMecanumDrive drive){
        trajectoryHomeToDetect = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseDetectRandomization)
                .build();

        drive.followTrajectory(trajectoryHomeToDetect);
    }

    private double detectShippingElementAndReturnYOffset(ShippingElementDetector shippingElementDetector, Arm arm){
        String location = shippingElementDetector.detectPringle();
        double yOffset = 0;

        if (ShippingElementDetector.RIGHT.equals(location)){
            arm.move(Arm.HIGH_POSITION);
            yOffset = -2;
        }
        else if (ShippingElementDetector.NEITHER.equals(location)){
            arm.move(Arm.LOW_POSITION);
            yOffset = -1;
        }

        return yOffset;
    }

    private void driveToShippingHub(SampleMecanumDrive drive, double yOffset){
        poseShippingHub = poseShippingHub.plus(new Pose2d(0, yOffset, 0));

        trajectoryDetectToShippingHub = drive.trajectoryBuilder(trajectoryHomeToDetect.end())
                .lineToLinearHeading(poseShippingHub)
                .build();

        drive.followTrajectory(trajectoryDetectToShippingHub);
    }

    private void driveToHome(SampleMecanumDrive drive){
        trajectoryShippingHubToHome = drive.trajectoryBuilder(trajectoryDetectToShippingHub.end())
                .lineToLinearHeading(poseHome)
                .build();

        drive.followTrajectory(trajectoryShippingHubToHome);
    }

    private void driveToWarehouse(SampleMecanumDrive drive){
        trajectoryHomeToWarehouse = drive.trajectoryBuilder(trajectoryShippingHubToHome.end())
                .lineToLinearHeading(poseWarehousePose)
                .build();

        drive.followTrajectory(trajectoryHomeToWarehouse);
    }
}