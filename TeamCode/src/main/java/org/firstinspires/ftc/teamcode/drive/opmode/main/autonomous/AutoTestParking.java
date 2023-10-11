package org.firstinspires.ftc.teamcode.drive.opmode.main.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TagDetector;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoTestParking extends LinearOpMode {
    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        detector = new TagDetector(hardwareMap, telemetry);
        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(37, -60, Math.toRadians(90)));
        //come to high junction
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(90)))
                //.forward(52)
                .forward(72)
                .build();
        Trajectory traj2= drive.trajectoryBuilder(new Pose2d(37, 12, Math.toRadians(90)))
                //.forward(52)
                .back(72)
                .build();
        Trajectory traj3= drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(90)))
                //.forward(52)
                .forward(24)
                .build();
        /**
         * Custom sleeve trajectories, 1-2-3 = left-middle-right
         */
        Trajectory traj_1= drive.trajectoryBuilder(new Pose2d(33, -36, Math.toRadians(90))) //y = 30
                .strafeLeft(22)
                .build();
        Trajectory traj_2= drive.trajectoryBuilder(new Pose2d(33, -36, Math.toRadians(90))) //y = 30
                .lineToLinearHeading(new Pose2d(37, -33, Math.toRadians(90)))
                .build();
        Trajectory traj_3= drive.trajectoryBuilder(new Pose2d(33, -36, Math.toRadians(90))) //y = 30
                .strafeRight(26)
                .build();

        drive.initEverythinginStandart();
        while (!isStopRequested() && !isStarted()){
            TagDetector.Tag tag = detector.getTag();
            if(tag != TagDetector.Tag.noTag)
                currTag = tag;
            telemetry.addLine("Tag: " + tag.name());
            telemetry.update();
            sleep(40);
        }
        waitForStart();

        if (isStopRequested()) return;


        //sleep(2500);
        //drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        if(currTag == TagDetector.Tag.right){
            drive.followTrajectory(traj_3);
        } else if(currTag == TagDetector.Tag.mid){
            drive.followTrajectory(traj_2);
        } else {
            drive.followTrajectory(traj_1);
        }
        /*
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        new Thread(() -> drive.EXTENDIntakePosition(0.79, extd)).start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        new Thread(() -> drive.GRAB_PUTCONEIntakePosition1());
        */

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }
}