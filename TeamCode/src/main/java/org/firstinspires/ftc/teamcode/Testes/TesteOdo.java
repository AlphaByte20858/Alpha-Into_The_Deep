package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name = "TesteOdo", group = "LinearOpMode")
public class TesteOdo extends LinearOpMode {
    private DcMotor par, perp;
    public void runOpMode(){

        par = hardwareMap.get(DcMotorEx.class, "MEF");
        perp = hardwareMap.get(DcMotorEx.class, "MDT");

        par.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        par.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(true){
            telemetry.addData("Valora roda esquerda:",perp.getCurrentPosition());
            telemetry.addData("Valora roda direita:",par.getCurrentPosition());
            telemetry.update();
        }

    }

}
