package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mechanum TeleOP", group="Linear Opmode")
public class MechanumTeleOp extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private MechanumClass drive = new MechanumClass();
    double power = .5;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap, false);


        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            double horizontal = -gamepad1.left_stick_y;
            double vertical = gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            if(gamepad1.dpad_up && power != 1)
            {
                power+=.25;
            }
            if(gamepad1.dpad_down && power != .25)
            {
                power-=.25;
            }

            drive.teleOP(power,pivot,vertical,horizontal);

            telemetry.addData("x1 encoder val", drive.getEncoderVal("x1"));
            telemetry.addData("x2 encoder val", drive.getEncoderVal("x2"));
            telemetry.addData("y encoder val", drive.getEncoderVal("y"));
            telemetry.addData("Front Left: ", -power * pivot + (power *(-vertical - horizontal)));
            telemetry.addData("Front Right: ", power * pivot + (power *(-vertical + horizontal)));
            telemetry.addData("Back Left: ", -power * pivot + (power *(-vertical + horizontal)));
            telemetry.addData("Back Right: ", power * pivot + (power * (-vertical - horizontal)));

            telemetry.addData("Pivot: ", pivot);
            telemetry.addData("Vertical: ", vertical);
            telemetry.addData("Horizontal: ", horizontal);
            telemetry.update();
        }


    }
}