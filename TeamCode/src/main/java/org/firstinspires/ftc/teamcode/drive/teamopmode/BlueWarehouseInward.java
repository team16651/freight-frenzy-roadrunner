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
public class BlueWarehouseInward extends LinearOpMode {

    Arm arm = null;
    DcMotor armMotor = null;
    Servo handServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo, true);


        Trajectory toShippingHub = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-2.105, -21.214, 4.354))
                .build();


        Trajectory toHome = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(0,0,0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory toWarehouse = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(44.698, -1.62))
                .build();

        Trajectory toParking = drive.trajectoryBuilder(toWarehouse.end())
//                .strafeLeft(26)
                .strafeTo(new Vector2d(44.698, -29.580))
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
        arm.move(Arm.HIGH_POSITION);
        this.sleep(500);
        drive.followTrajectory(toShippingHub);
        arm.release();
        this.sleep(1500);
        drive.followTrajectory(toHome);
        this.sleep(500);
        drive.followTrajectory(toWarehouse);
        this.sleep(500);
        drive.followTrajectory(toParking);
        arm.move(Arm.PARK_POSITION);

//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(trajectoryForward);
//            drive.followTrajectory(trajectoryBackward);
//        }
    }
}