package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="LEFT BLUE", group="Linear Opmode")
public class AutoCOMP_LeftBlue extends LinearOpMode
{
    MechanumClass mc = new MechanumClass();
    //AprilTagClass aTag = new AprilTagClass();

    IMUClass imu = new IMUClass();
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
        //mc.backClawClose(false,1000);

        if(opModeIsActive())
        {
            mc.backClawClose(true,2000);

            mc.backClawClose(false,2000);

            /*
            int pixelPosition = 0;

            mc.backClawClose(true);

            //Raise Camera to look at pixel
            mc.liftSlide(.5,1000,4000);
            //Move forward if necessary
            mc.drive(90,.5,1500,500,true);
            //Scan
            while(opModeIsActive())
            {
                if(pixelPosition == 0)
                {
                    //Update pixelPosition variable
                    pixelPosition = mc.returnPixelRegion(cam);
                }
                else
                {
                    break;
                }
            }

            //Retract slides to position 0
            mc.liftSlide(.5,0,3000);
            //Move forward
            mc.drive(90,.5,1500,500,true);

            //Rotate 180 degrees
            mc.rotate(180,.5,5000,imu);
            //Move back and place pixel on right marker
            mc.backClawClose(false);

            //Go park
            mc.drive(0,.5,5000,5000,true);


             */
            /*

            int pixelPosition = 0;
            boolean driveBool = true;

            mc.liftSlide(.5,1000, 5000);

            mc.rotateArm(.9);

            mc.closeClaw(true);

            mc.drive(0,.5,3000,1000,true);

            mc.rotate(90,.5,5000,imu);
            mc.rotate(-90,.5,5000,imu);

            mc.closeClaw(false);

            mc.liftSlide(.5,0,5000);
            */
        }

    }
}
