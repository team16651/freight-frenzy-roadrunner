package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class BlueCarouselWarehouse extends LinearOpMode {

    CarouselSpinner carouselSpinner = null;
    DcMotor carouselSpinnerMotor = null;

    Arm arm = null;
    DcMotor armMotor = null;
    Servo handServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

//        Trajectory toCarouselSpinner = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d( 0,-13.570))
//                .build();
//
//        Trajectory toCarouselSpinner2 = drive.trajectoryBuilder(toCarouselSpinner.end())
//                .lineToLinearHeading(new Pose2d(-0.1, -13.570))
//                .build();

        Trajectory toCarouselSpinner = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d( 0,-3.5))
                .build();



        Trajectory toCarouselSpinner2 = drive.trajectoryBuilder(toCarouselSpinner.end())
                .lineToLinearHeading(new Pose2d(-5.1, -3.5, 5.6),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();




        Trajectory toShippingMid = drive.trajectoryBuilder(toCarouselSpinner2.end())
                .lineToLinearHeading(new Pose2d(36.049, -3.5, 5.05))
                .build();

        Trajectory toShippingMid2 = drive.trajectoryBuilder(toShippingMid.end())
                .lineToLinearHeading(new Pose2d(36.049, -19.385, 5.05))
                .build();

        Trajectory toPark = drive.trajectoryBuilder(toShippingMid.end())
                .lineToLinearHeading(new Pose2d(92.466, -3.777, 0))
                .build();
//       new TrajectoryBuilder(new Pose2d())
//  .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
//  .build()



//        Trajectory toParking = drive.trajectoryBuilder(toHome.end())
////                .strafeTo(new Vector2d(75.466, -0.73))
////                .forward(72.393)
//                .lineToLinearHeading(new Pose2d(72.393, 0.04, Math.toRadians(-15)),
//                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//                .build();



//        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
//                .forward(DISTANCE)
//                .build();
//
//        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
//                .back(DISTANCE)
//                .build();

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
        drive.followTrajectory(toShippingMid);
        this.sleep(500);
        drive.followTrajectory(toShippingMid2);
        arm.release();
        this.sleep(1500);
        drive.followTrajectory(toShippingMid);
        this.sleep(500);
        drive.followTrajectory(toPark);

//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(trajectoryForward);
//            drive.followTrajectory(trajectoryBackward);
//        }
    }
}