package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Testing Autonomous", group="Linear Opmode")
public class CompetitionAutonomous extends LinearOpMode
{
    MechanumClass mc = new MechanumClass();
    //AprilTagClass aTag = new AprilTagClass();

    public IMUClass imu = new IMUClass();
    //TensorflowClass tensorflow = new TensorflowClass();
    CameraClass cam = new CameraClass();
    @Override
    public void runOpMode() throws InterruptedException
    {
        mc.init(hardwareMap, true);
        //aTag.initAprilTag(hardwareMap);
        imu.initIMU(hardwareMap);
        cam.init(hardwareMap);
        //tensorflow.initTfod(hardwareMap);
        waitForStart();

        if(opModeIsActive())
        {
            mc.drive(90,.5,2000,500,true);
            //mc.drive(90,.5,2000,500,true);
            //mc.drive(180,.5,2000,500,true);
            //mc.drive(270,.5,2000,500,true);

            //mc.rotate(90,.5,5000,imu);
            //mc.rotate(3.14/2,.5,5000,imu);

            //mc.drive(90,.5,2000,500,true);

            mc.rotate(90,.5,5000,imu);

            /*
            int pixelPosition = 0;
            boolean driveBool = true;

            mc.liftSlide(.5,1000, 5000);

            mc.rotateArm(.9,1000);

            mc.closeClaw(true, 1000);

            mc.drive(0,.5,3000,1000,true);

            mc.rotate(90,.5,5000,imu);
            mc.rotate(-90,.5,5000,imu);

            mc.closeClaw(false, 1000);

            mc.liftSlide(.5,0,5000);


             */
        }

    }
}
