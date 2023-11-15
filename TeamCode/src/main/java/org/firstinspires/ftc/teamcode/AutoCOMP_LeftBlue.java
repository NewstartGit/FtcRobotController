package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="((COMP)) LEFT BLUE", group="Linear Opmode")
public class AutoCOMP_LeftBlue extends LinearOpMode
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
        //mc.backClawClose(false,1000);

        if(opModeIsActive())
        {
            mc.backClawClose(true,500);

            //mc.backClawClose(false,2000);

            mc.closeClaw(true,500);

            mc.drive(90,.5,4000,3000,true);

            mc.drive(180,.5,4000,1750,true);

            mc.closeClaw(false,1);

            mc.drive(270,.5,4000,1000,true);

            mc.drive(180,.5,4000,5000,true);

            mc.backClawClose(false,1);

            mc.drive(90,.5,4000,1000,true);

            //NOT WORKING :((
            /*

            int pixelPosition = 0;

            mc.backClawClose(false,250);
            mc.backClawClose(true,500);

            //Raise Camera to look at pixel
            mc.liftSlide(.25,2000,4000);

            mc.rotateArm(.9,500);

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

            mc.rotateArm(1,500);
            if(pixelPosition == 2)
            {
                mc.drive(90,.5,2000,2000,true);
                mc.rotate(90,.5,2000,imu);

                mc.drive(0,.5,1500,750,true);
                mc.drive(270,.5,2000,2000,true);

                mc.backClawClose(false,1);
                //Thread.sleep(1000);

                mc.drive(90,.5,1500,1500,true);

                mc.drive(0,.5,4000,5000,true);

            }


            /*
            //Move forward
            mc.drive(90,.5,1500,500,true);

            //Rotate 180 degrees
            mc.rotate(180,.5,5000,imu);
            //Move back and place pixel on right marker
            mc.backClawClose(false,1000);

            //Go park
            mc.drive(0,.5,5000,5000,true);



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
