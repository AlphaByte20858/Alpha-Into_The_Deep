package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "AutoSample", group = "LinearOpMode")
public class AutoTestes extends LinearOpMode {
    DcMotorEx MET, MEF, MDF, MDT;
    public void runOpMode() {
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");

        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);

        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive movi = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        Action açao;

        açao = movi.actionBuilder(movi.pose)
                .splineToConstantHeading(new Vector2d(10, 5), Math.toRadians(180))
                .build();

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                açao
        ));
    }
}
