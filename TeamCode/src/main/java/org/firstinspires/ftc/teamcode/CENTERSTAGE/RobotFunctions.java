package org.firstinspires.ftc.teamcode.CENTERSTAGE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotFunctions {
    DcMotor sliderRight;
    DcMotor sliderLeft;
    Servo pivotServo;
    Servo clawServo;
    Servo backServoLeft;
    Servo backServoRight;

    CRServo droneServo;
    public RobotFunctions(HardwareMap hwMap)
    {
        sliderRight = hwMap.get(DcMotor.class, "R_Slider");
        sliderLeft = hwMap.get(DcMotor.class, "L_Slider");

        pivotServo = hwMap.get(Servo.class, "PIVOT_Servo");
        clawServo = hwMap.get(Servo.class, "CLAW_Servo");
        backServoLeft = hwMap.get(Servo.class, "BL_Servo");
        backServoRight = hwMap.get(Servo.class,"BR_Servo");

        droneServo = hwMap.get(CRServo.class,"DRONE_Servo");

        sliderRight.setDirection(DcMotor.Direction.REVERSE);

        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftSlide(double power, int position, long delay) throws InterruptedException
    {
        sliderLeft.setTargetPosition(-position);
        sliderRight.setTargetPosition(-position);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderLeft.setPower(power);
        sliderRight.setPower(power);

        Thread.sleep(delay);
    }
    public void rotateArm(double servoPosition, long delay) throws InterruptedException
    {
        //1 is default, .9 is slightly up, .75 is for board
        pivotServo.setPosition(servoPosition);

        Thread.sleep(delay);
    }
    public void closeClaw(boolean clawState, long delay) throws InterruptedException
    {
        if(clawState)
        {
            clawServo.setPosition(1);
        }
        else
        {
            clawServo.setPosition(0);
        }
        Thread.sleep(delay);
    }
    public void backClawClose(boolean clawState,long delay) throws InterruptedException
    {
        if(clawState)
        {
            backServoLeft.setPosition(.25);
            backServoRight.setPosition(.25);
        }
        else
        {
            backServoLeft.setPosition(0);
            backServoRight.setPosition(0);
        }
        Thread.sleep(delay);
    }

}
