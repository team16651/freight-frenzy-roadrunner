package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class RedCarouselStorage extends LinearOpMode {

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
        arm = new Arm(armMotor, handServo, true);

        Trajectory toCarouselSpinner = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-15.000, 5.355))
                .build();

        Trajectory toShippingHub = drive.trajectoryBuilder(toCarouselSpinner.end())
                .lineToLinearHeading(new Pose2d(21.712, 9.811, 1.427),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory toShippingHub2 = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(23.924, 22.819, 1.427))
                .build();

        Trajectory toPark = drive.trajectoryBuilder(toShippingHub2.end())
                .lineToLinearHeading(new Pose2d(21.712, 9.811, 1.427))
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark.end())
                .lineToLinearHeading(new Pose2d(-21.066, 27.936, 0))
                .build();




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
        carouselSpinner.spin(true, 0.5);
        this.sleep(2500);
        carouselSpinner.stop();
        this.sleep(500);
        drive.followTrajectory(toShippingHub);
        this.sleep(500);
        drive.followTrajectory(toShippingHub2);
        arm.release();
        this.sleep(500);
        drive.followTrajectory(toPark);
        this.sleep(500);
        drive.followTrajectory(toPark2);
        arm.move(Arm.PARK_POSITION);

//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(trajectoryForward);
//            drive.followTrajectory(trajectoryBackward);
//        }
    }
}