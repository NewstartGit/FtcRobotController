package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="testMechanumONE", group="Linear Opmode")
public class testMechanumONE extends LinearOpMode
{
    MechanumClass mc = new MechanumClass();
    AprilTagClass aTag = new AprilTagClass();

    IMUClass imu = new IMUClass();
    TensorflowClass tensorflow = new TensorflowClass();
    @Override
    public void runOpMode() throws InterruptedException
    {
        mc.init(hardwareMap, true);
        aTag.initAprilTag(hardwareMap);
        imu.initIMU(hardwareMap);
        tensorflow.initTfod(hardwareMap);
        waitForStart();

        if(opModeIsActive())
        {
            int pixelPosition = 0;
            //Move forward to get better view of pixels
            mc.drive(90,.5,3000,2000,true);

            //Search for and return pixel
            while(opModeIsActive())
            {
                if(pixelPosition == 0)
                {
                    pixelPosition = mc.returnPixelRegion(tensorflow);
                }
            }
            /*
            boolean driveBool = true;

            //mc.alignWithAprilTag(.25,30,aTag);


            mc.drive(90,.5,3000,3000,true);
            mc.rotate(90,.25,3000,imu);
            mc.drive(90,.5,3000,1500,true);




            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,10,aTag,3);
                }
                else
                {
                    break;
                }
                //mc.drive(270,0.25,1000,1000,true);
            }
            driveBool = true;

            mc.drive(270,.5,6000,3000,true);
            */
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
