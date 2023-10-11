package org.firstinspires.ftc.teamcode.drive.opmode.main.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.IntakeConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;
    Servo sSlider, sRotator, sFlipperR, sFlipperL, sClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        sClaw = hardwareMap.servo.get("Claw");
        sSlider = hardwareMap.servo.get("Slider");
        sFlipperR = hardwareMap.servo.get("Flipper_right");
        sFlipperL = hardwareMap.servo.get("Flipper_left");
        sRotator = hardwareMap.servo.get("Rotator");

        sSlider.setPosition(IntakeConstants.SLIDER_CLOSE);
        sClaw.setPosition(IntakeConstants.CLAW_CLOSE);
        sFlipperL.setPosition(IntakeConstants.FLIPPER_LEFT_STANDART);
        sFlipperR.setPosition(IntakeConstants.FLIPPER_RIGHT_STANDART);
        sRotator.setPosition(IntakeConstants.ROTATOR_UPSIDE_DOWN);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
    }
}