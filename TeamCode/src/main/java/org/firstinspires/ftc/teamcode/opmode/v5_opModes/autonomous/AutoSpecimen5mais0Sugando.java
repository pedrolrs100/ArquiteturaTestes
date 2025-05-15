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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDeDecisao.Controladora5mais0_sugando;
import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.UpperSubsystemStates;

@Disabled
@Config
@Autonomous(name = "AutonomoSpecimen5+0 beta ", group = "Autonomous")
public class AutoSpecimen5mais0Sugando extends LinearOpMode {
    V5 robot;
    Action push;
    public static int target = 0;
    private boolean encerrar;
    private Action proximaAction;
    public static double
            //todo depositos
            Xdeposit1 = -49 ,    Ydeposit1 = -57 ,  Hdeposit1 = 35,
            Xdeposit2 = -55 ,    Ydeposit2 = -53.3,  Hdeposit2 = 45,
            //todo colects
            XCollect2 = 35 , YCollect2 = -40, HCollect2 = 60, //todo cansado okey
            XCollect3 = 45 ,   YCollect3 = -40 , HCollect3 = 60, //todo cansado okey
            XCollect4 = 50 ,   YCollect4 = -35 , HCollect4 = 60, //todo okey
            //todo ReadyCollect
            XReadCollect2 = 35 , YReadCollect2 = -40, HReadCollect2 = 60, //todo cansado okey
            XReadCollect3 = 45 ,   YReadCollect3 = -40 , HReadCollect3 = 60, //todo cansado okey
            XReadCollect4 = 50 ,   YReadCollect4 = -35 , HReadCollect4 = 50, //todo cansedo okey

            //todo ejetar
            XEntregarSample1 = 35 , YEntregarSample1 = -45 , HEntregarSample1 = -60, //todo cansado okey
            XEntregarSample2 = 35 , YEntregarSample2 = -45 , HEntregarSample2 = -60, //todo cansado okey

            XEntregarSampl3 = 45 , YEntregarSample3 = -45 , HEntregarSample3 = -60, // todo cansado okey

            XEntregarSample4 = 40 , YEntregarSample4 = -45 , HEntregarSample4 = -70 //todo cansado okey


    ;



                                ;


    @Override
    public void runOpMode()  {



        robot = Controladora5mais0_sugando.create5mais0Robot(hardwareMap,telemetry);
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToInitialSpecimen(),
                        robot.intakeInferior.intakeSuccao.GoToInitial()
                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
                        //robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );
        //push = pushSamples();
        waitForStart();
        resetRuntime();
        while (robot.controladoraSPECIMEN.sampleID <= 2) {
            encerrar = false;
            proximaAction = robot.controladoraSPECIMEN.decideProximaAcao(robot, getRuntime());
            if(proximaAction != null){
                Actions.runBlocking(
                        new ParallelAction(
                                //todo Roda a próxima ação retornada pela controladora em paralelo com nossas automatizações
                                AutomationSpecimen(),
                                new SequentialAction(
                                        proximaAction,
                                        new InstantAction(() -> encerrar = true)
                                )

                        )
                );
            }

        }

        // todo: reseta no final
        encerrar = false;
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
        Actions.runBlocking(
                AutomationSpecimen()
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
                telemetryPacket.addLine("quantas vezes mandou a próxima ação: " + robot.controladoraBASKET.quantasVezesFoiExecutado);
                if(encerrar){
                    return false;
                }
                return true;

            }
        };
    }

    public Action goToDeposit(){
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .strafeTo(new Vector2d(-6,-22))
                //.splineToConstantHeading(new Vector2d(-6, -26), Math.toRadians(90))
                //todo: Go to empurrar sample 1
                .build();

    }
    public Action pushSamples() {
        return
                robot.md.actionBuilder(robot.md.pose)

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-85)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(44, -5, Math.toRadians(-90)), Math.toRadians(0))

                        .setTangent(Math.toRadians(-90))
                        .lineToY(-58)

                        //todo empurrar sample 2

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(56, -5, Math.toRadians(-95)), Math.toRadians(-90))



                        .lineToY(-57)
                        .setTangent(Math.toRadians(90))
                        //.strafeToConstantHeading(new Vector2d(47, -63))
                        .splineToLinearHeading(new Pose2d(43, -65, Math.toRadians(-90)), Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(55, -45, Math.toRadians(-90)), Math.toRadians(-85))
                        .waitSeconds(0.5)
                        //.lineToY(-66)

                        .build();


    }
    public Action goToDeposit2() {
        return robot.md.actionBuilder(new Pose2d(51, -63, Math.toRadians(-90)))
                //todo: colocar primeiro specimen
                //.strafeTo(new Vector2d(-6,-22))
                .strafeTo(new Vector2d(-12,-17))
                //.splineToSplineHeading(new Pose2d(EIXOX,-27,Math.toRadians(-90)
                .build();
    }
    public Action goToDeposit3() {
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(0, -30.5), Math.toRadians(90))
                .build();
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
                        .strafeTo(new Vector2d(-5,-40))
                        .build()
        );

    }
    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }
    public Action deposit2() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit2()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }


    public Action intake(double x , double y){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                positionIntake(),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(2.60).build(),
                                collect()
                        ),
                        robot.md.actionBuilder(new Pose2d(x, y, Math.toRadians(-90)))
                                //.strafeTo(new Vector2d(38 ,-35))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(51, -55, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(0.4)
                                .lineToY(-66)
                                .build()
                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.30).build()


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
    public Action collect2() {
        return new SequentialAction(
                //new InstantAction(()->{robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(2);}),
                robot.md.actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.8).build()
                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build()
        );
    }

    public Action sample1() {
        return new SequentialAction(
                new ParallelAction(
                        deposit(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )

                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }
    public Action sample2() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        goToPark(),
                        new SequentialAction(
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.45).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)
                        )

                )


        );
    }

}