package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@TeleOp (name = "E4T Encoder Test", group = "")
public class E4TEncoderTest extends LinearOpMode
{
    AnalogInput encoder = null;

    @Override
    public void runOpMode()
    {
        encoder = hardwareMap.analogInput.get("encoder");
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("Raw Value", encoder.getVoltage());
            telemetry.update();
        }
    }
}
