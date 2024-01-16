package org.firstinspires.ftc.teamcode.CENTERSTAGE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Roadrunner Backstage Blue", group="Linear Opmode")
public class RoadrunnerAutoBACKSTAGE_BLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotFunctions robot = new RobotFunctions(hardwareMap);
        HuskyLensClass husky = new HuskyLensClass();
        husky.init(hardwareMap);

        int pixelPosition = 3;//husky.runHusky();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-62, 12, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Place Pixel position = 1
        Trajectory leftSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(-34,30,Math.toRadians(90)))
                .build();

        //Place Pixel position = 2
        Trajectory rightSpike = drive.trajectoryBuilder(leftSpike.end(), false)
                .back(22)
                .build();

        //Place Pixel position = 3
        Trajectory centerSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(-24.5,18,Math.toRadians(90)))
                .build();


        robot.closeClaw(true,500);
        robot.rotateArm(.25,1000);
        robot.backClawClose(true, 100);


        waitForStart();

        if(isStopRequested()) return;

        switch(pixelPosition)
        {
            case 1:
                drive.followTrajectory(leftSpike);
                robot.backClawClose(false,1000);
                break;
            case 2:
                drive.followTrajectory(leftSpike);
                drive.followTrajectory(rightSpike);
                //robot drop pixel
                robot.backClawClose(false, 1000);
                break;
            case 3:
                drive.followTrajectory(centerSpike);
                //robot drop pixel
                robot.backClawClose(false, 1000);
                break;
            default:
                break;
        }


    }
}