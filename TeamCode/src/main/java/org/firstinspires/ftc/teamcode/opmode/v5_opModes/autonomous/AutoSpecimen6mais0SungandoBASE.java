package org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.UnderGrounSubystemStates;


@Config
@Autonomous(name = " 6 + 0 NOVO ", group = "Autonomous")
public class AutoSpecimen6mais0SungandoBASE extends LinearOpMode {

    V5 robot;
    public boolean encerrar = false;

    public static double delayFecharGarraIntake = 1.65, velIntake = 35, posIntake = -65;

    @Override
    public void runOpMode()  {

        //todo antiga: Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));
        Pose2d initialPose = new Pose2d(8, -61, Math.toRadians(-90));

        robot = new V5(hardwareMap,telemetry);
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -80;
        MecanumDrive.PARAMS.maxWheelVel  = 80;

        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.setPosition(0.209)),
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.528);}),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToInitialSpecimen()
                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
                        //robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        AutomationSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                                new InstantAction(()-> {robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;}),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(5).build(),
                                new InstantAction(()-> {robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;}),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(5).build(),
                                new InstantAction(()-> {robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.EJETING;})),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(5).build(),
                                new InstantAction(()-> {robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;}),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(5).build()

                )
        );

    }



    public Action AutomationSpecimen(){
        return new Action() {
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    started = true;
                }

                robot.outtakeIntakeSuperior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitorAutonomo(telemetryPacket);
                robot.runStatesSpecimenAutonomo(robot.carteiro, getRuntime(),robot);
                telemetryPacket.addLine("Power Sugador: "+ robot.intakeInferior.intakeSuccao.sugador.getPower());
                telemetryPacket.addLine("tempo de execução: "+ getRuntime());
                telemetryPacket.addLine("sampleID: "+ robot.controladoraBASKET.sampleID);
                telemetryPacket.addLine("estadoAutonomo: "+ robot.controladoraBASKET.estadoSample);
                telemetryPacket.addLine("posição intake:"+ robot.intakeInferior.intakeSuccao.alcapao.getPosition());
                telemetryPacket.addLine("quantas vezes mandou a próxima ação: " + robot.controladoraBASKET.quantasVezesFoiExecutado);
                if(encerrar){
                    return false;
                }
                return true;

            }
        };
    }

    public Action firstIntake(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //.strafeTo(new Vector2d(38 ,-35))
                        .lineToY(-63,  new TranslationalVelConstraint(30.0))
                        .build(),

                collect()

        );
    }
    public Action goToPark() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeTo(new Vector2d(-5,-40)/* ,null, new ProfileAccelConstraint(-200, 200)*/)
                        .build()
        );

    }

    public  Action positionIntake() {
        return new ParallelAction(
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()                                                                                                                                                                                                           ,
                robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                robot.outtakeIntakeSuperior.garraSuperior.goToIntakeSpecimen(),
                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(-100)


        );
    }
    public Action collect() {
        return new SequentialAction(
                robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.45).build()
        );
    }




    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        goToPark(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                                robot.outtakeIntakeSuperior.braco.goToInital(),
                                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)
                        )

                )


        );
    }

}