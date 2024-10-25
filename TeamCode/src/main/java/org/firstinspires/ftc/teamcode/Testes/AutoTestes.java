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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "AutoSample", group = "LinearOpMode")
public class AutoTestes extends LinearOpMode {
    DcMotorEx MET, MEF, MDF, MDT;
    Servo servinho;
    ElapsedTime autonomus;

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

        autonomus.reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive movi = new MecanumDrive(hardwareMap, new Pose2d(12, -63, Math.toRadians(0)));

        Action direita1, direita2, direita3;

        direita1 = movi.actionBuilder(movi.pose)
                .lineToXConstantHeading(4)
                .lineToYSplineHeading(-34, Math.toRadians(-90))
                .build();

        direita2 = movi.actionBuilder(new Pose2d(4, 34, Math.toRadians(-90)))
                .lineToXConstantHeading(26)
                .splineToConstantHeading(new Vector2d(63, -15), Math.toRadians(270))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(62, -62), Math.toRadians(-90))
                .build();

        direita3 = movi.actionBuilder(new Pose2d(62, -62, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -14), Math.toRadians(-90))
                .splineTo(new Vector2d(50, -62), Math.toRadians(-90))
                .waitSeconds(3)
                .lineToY(-48)
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                direita1
        ));
        servinho.setPosition(1);
        Actions.runBlocking(new SequentialAction(
                direita2,
                direita3
        ));
    }
}