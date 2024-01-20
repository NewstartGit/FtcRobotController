package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MechanumClass;

@Disabled

@Autonomous(name="testMechanumONE", group="Linear Opmode")
public class testMechanumONE extends LinearOpMode
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

        if(opModeIsActive())
        {
            int pixelPosition = 0;
            //Move forward to get better view of pixels
            mc.drive(90,.5,5000,1200,true);
            //mc.rotate(90,.5,3000,imu);
            mc.drive(0,.5,5000,5000,true);
            mc.rotate(90,.5,10000,imu);
            //mc.drive(180,.5,10000,5000,true);

            //mc.drive(90,0,0,0,false);
            //Search for and return pixel
            /*
            while(opModeIsActive())
            {
                if(pixelPosition == 0)
                {
                    pixelPosition = mc.returnPixelRegion(cam);
                }
                else
                {
                    break;
                }
            }

            boolean driveBool = true;

            //mc.alignWithAprilTag(.25,30,aTag);


            mc.drive(90,.5,3000,750,true);
            mc.rotate(90,.225,6000,imu);
            mc.drive(90,.5,4000,2000,true);

            mc.drive(0,.5,2000,1000,true);



            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,15,cam,pixelPosition);
                }
                else
                {
                    break;
                }
                //mc.drive(270,0.25,1000,1000,true);
            }

            driveBool = true;

            mc.drive(270,.5,6000,3000,true);

            /*
            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,10,aTag,2);
                }
                else
                {
                    break;
                }
                //mc.drive(270,0.25,1000,1000,true);
            }

            driveBool = true;

            mc.drive(270,.5,6000,3000,true);
            /*
            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,10,aTag,1);
                }
                else
                {
                    break;
                }
                //mc.drive(270,0.25,1000,1000,true);
            }
            driveBool = true;

            mc.drive(270,.5,6000,3000,true);

            //mc.drive(90,.5,3000,1000,true);
            /*
            mc.rotate(90,.25,5000, imu);
            mc.drive(0,.5,3000,3000,true);


             */
            /*
            boolean driveBool = true;

            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,40,aTag);
                }
            }/*


            boolean driveBool = true;
            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.strafeAroundAprilTag(.1,40,aTag);
                }
            }
            */


        }

    }
}
