package org.firstinspires.ftc.teamcode.robot.Controller;

import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.Hdeposit1;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.XCollect2;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.Xdeposit1;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.YCollect2;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.Ydeposit1;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.tangentCollect4;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red;
import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraOpeningStates;

import java.util.ArrayList;

public class Controladora {
    double limelightCorrection = 0;
    public AutonomoStates estadoSample = AutonomoStates.INITIAL;
    public int sampleID = 1, samplesNotCollected = 0;
    ArrayList<Integer> samplesIdNotCollected = new ArrayList<>();


    public int quantasVezesFoiExecutado = 0;

    // todo: verificar estados
    public Controladora () {

    }
    public static V5 create0mai5Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        Pose2d initialPose = new Pose2d(-35,  -62, Math.toRadians(0));
        V5 robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -60;
        MecanumDrive.PARAMS.maxWheelVel  = 80;

        return robot;
    }

    public Action decideProximaAcao(V5 robot, double runtime){
        quantasVezesFoiExecutado++;
        robot.md.updatePoseEstimate();
        Action proximaAction = null;
        if(estadoSample == AutonomoStates.INITIAL && sampleID == 1 ){
            //todo adicionar ID
            proximaAction = depositAndGoToReadyToCollect(1,robot);
        }
        else if(estadoSample == AutonomoStates.COLLECT && sampleID >= 2){
            if(robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake()){
                proximaAction = depositAndGoToReadyToCollect(sampleID, robot);
            }
            else {
                samplesNotCollected++;
                samplesIdNotCollected.add(sampleID);
                sampleID++;

                proximaAction = goToReadyToCollect(sampleID , robot);
                if(sampleID > 4){// todo: dont go to sub with "goReadyCollectS5"
                    proximaAction = new InstantAction(() -> {});
                }
            }
        }
        else if(estadoSample == AutonomoStates.DEPOSIT && sampleID >= 2){
            proximaAction = goToReadyToCollect(sampleID, robot);
        }
        else if(estadoSample == AutonomoStates.READY_TO_COLLECT && sampleID >= 2){
            proximaAction = CollectSample(sampleID,robot);
        }
        // todo: caso dê errado
        return proximaAction;
    }
    public Action depositAndGoToReadyToCollect(int idSample, V5 robot){
        robot.md.updatePoseEstimate();
        if(idSample == 1){
            return depositSample1(robot.md.pose, robot);
        }
        if(idSample == 2){
            return depositSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return depositSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return depositSample4(robot.md.pose, robot);
        }
        if(idSample == 5){
            return depositSample5(robot.md.pose, robot);
        }

        // default
        return new SequentialAction();
    }
    public Action CollectSample(int idSample, V5 robot){
        estadoSample = AutonomoStates.COLLECT;
        robot.md.updatePoseEstimate();
        if(idSample == 2){
            return GoToCollectSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return GoToCollectSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return GoToCollectSample4(robot.md.pose, robot);
        }
        if(idSample == 5) {
            robot.lastSample = true;
            return GoToCollectSample5(robot.md.pose, robot);
        }
        // default
        return new SequentialAction();
    }
    public Action goToReadyToCollect(int idSample, V5 robot){ /*todo: para quando não conseguir pegar nenhuma sample, ir pra outra*/
        estadoSample = AutonomoStates.READY_TO_COLLECT;

        robot.md.updatePoseEstimate();
        if(idSample == 2){
            return GoToReadyToCollectSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return GoToReadyToCollectSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return GoToReadyToCollectSample4(robot.md.pose, robot);
        }
        // default
        return new SequentialAction();
    }





/************************************\
*   Ações específicas do 0 mais 5
\*************************************/

    /************************************\
     *   Deposits
     \*************************************/
    public Action depositSample1(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDepositSample1(robot.md.pose, robot),
                esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample1, robot),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.DEPOSIT),
                        new InstantAction(() -> sampleID += 1),
                        new InstantAction(() -> estadoSample = AutonomoStates.READY_TO_COLLECT),
                        GoToReadyToCollectSample2(robot.md.pose, robot)


                )
        );

    }
    public Action depositSample2(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        
                        new InstantAction(() -> estadoSample = AutonomoStates.READY_TO_COLLECT),
                        GoToReadyToCollectSample3(robot.md.pose, robot),
                    new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                )
        );

    }
    public Action depositSample3(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.READY_TO_COLLECT),
                        GoToReadyToCollectSample4(robot.md.pose, robot)
                )
        );

    }

    public Action depositSample4(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit4(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample4, robot),
                        new SequentialAction(
                               // esperarPeloOutakeTerSubidoEextender(1, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.READY_TO_COLLECT),
                        GoToReadyToCollectSample5(robot.md.pose, robot)
                )
        );

    }
    public Action depositSample5(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit5(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample4, robot),
                        new SequentialAction(
                                //esperarPeloOutakeTerSubidoEextender(1, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(

                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.READY_TO_COLLECT),
                        GoToReadyToCollectSample5(robot.md.pose, robot)
                )
        );

    }

    /************************************\
     *   Trajetória dos Deposits
     \*************************************/
    public Action goToDepositSample1(Pose2d pose2d, V5 robot){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)), Math.toRadians(-135), null, new ProfileAccelConstraint(-30, 30))
                //.waitSeconds(0.1)
                //.splineToLinearHeading(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)), Math.toRadians(-135))
                .build();

    }
    public Action goToDeposit2(Pose2d pose2d, V5 robot){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(AutSample0mais4Red.Xdeposit2, AutSample0mais4Red.Ydeposit2, Math.toRadians(AutSample0mais4Red.Hdeposit2)), Math.toRadians(-135))
                .build();

    }
    public Action goToDeposit4(Pose2d pose2d, V5 robot){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(80))
                //.splineToLinearHeading(new Pose2d(AutSample0mais4Red.Xdeposit2 - 5, AutSample0mais4Red.Ydeposit2 - 2, Math.toRadians(AutSample0mais4Red.Hdeposit2)), Math.toRadians(AutSample0mais4Red.tangentDeposit4))
                .splineToLinearHeading(new Pose2d(AutSample0mais4Red.Xdeposit4, AutSample0mais4Red.Ydeposit4, Math.toRadians(AutSample0mais4Red.Hdeposit4)), Math.toRadians(AutSample0mais4Red.tangentDeposit4))
                .build();

    }
    public Action goToDeposit5(Pose2d pose2d, V5 robot){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(robot.md.pose.position.x - 20, robot.md.pose.position.y + 20, Math.toRadians(0)), Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(AutSample0mais4Red.Xdeposit5, AutSample0mais4Red.Ydeposit5, Math.toRadians(AutSample0mais4Red.Hdeposit5)), Math.toRadians(-135))
                .build();

    }

    /******************************************\
     *   Trajetórias - GoToReadyToCollect (Padrão)
    \*******************************************/
    public Action GoToReadyToCollectSample2(Pose2d pose2d, V5 robot) {//todo okey

        Action move1 = robot.md.actionBuilder(pose2d)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2 - 12,Math.toRadians(AutSample0mais4Red.HCollect2    )), Math.toRadians(90))
                .build();

        return new SequentialAction(
         move1,
         LimelightCalculateCorrection(-0.82, robot)
        );


    }
    public Action GoToReadyToCollectSample3(Pose2d pose2d, V5 robot) {//todo okey
        return robot.md.actionBuilder(pose2d)
                // collect Sample 3
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(AutSample0mais4Red.XCollect3, AutSample0mais4Red.YCollect3 - 10, Math.toRadians(AutSample0mais4Red.HCollect3 - 5)), Math.toRadians(90))
                .build();

    }
    public Action GoToReadyToCollectSample4(Pose2d pose2d, V5 robot) {//todo okey
        return robot.md.actionBuilder(pose2d)
                // collect Sample 4
                .setTangent(Math.toRadians(AutSample0mais4Red.HCollect4))
                .splineToLinearHeading(new Pose2d(AutSample0mais4Red.XCollect4 + 10, AutSample0mais4Red.YCollect4 - 10, Math.toRadians(AutSample0mais4Red.HCollect4)), Math.toRadians(tangentCollect4))
                .build();
    }
    public Action GoToReadyToCollectSample5(Pose2d pose2d, V5 robot) {//todo okey
        return new SequentialAction(
                new InstantAction(() -> {
                    IntakeSuccao.power_Sugador = 0.7;
                }),
                robot.md.actionBuilder(pose2d)
                        .setTangent(Math.toRadians(AutSample0mais4Red.TangentCollect5))
                        .splineToLinearHeading(new Pose2d(AutSample0mais4Red.XCollect5, AutSample0mais4Red.YCollect5, Math.toRadians(AutSample0mais4Red.HCollect5)), Math.toRadians(AutSample0mais4Red.TangentCollect5part2))
                        .build()
        );
    }


    /******************************************\
     *   Trajetórias - GoToCollect
    \*******************************************/
    public Action GoToCollectSample2(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                        //.splineToSplineHeading(new Pose2d(XCollect2, YCollect2, Math.toRadians(AutSample0mais5.HCollect2)), Math.toRadians(90), null, new ProfileAccelConstraint(-20, 20))
                        .strafeToLinearHeading(new Vector2d(XCollect2 + limelightCorrection, YCollect2 - 4), Math.toRadians(AutSample0mais4Red.HCollect2 + 7),null, new ProfileAccelConstraint(-10, 10))
                        .strafeToLinearHeading(new Vector2d(XCollect2 + limelightCorrection, YCollect2 - 2), Math.toRadians(AutSample0mais4Red.HCollect2 - 7),null, new ProfileAccelConstraint(-10, 10))
                        .strafeToLinearHeading(new Vector2d(XCollect2 + limelightCorrection, YCollect2), Math.toRadians(AutSample0mais4Red.HCollect2),null, new ProfileAccelConstraint(-10, 10))
                        .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoOutake(2.5, robot)
        );
    }
    public Action GoToCollectSample3(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3 - 4, Math.toRadians(AutSample0mais4Red.HCollect3 + 3)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3 - 2, Math.toRadians(AutSample0mais4Red.HCollect3 - 7)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3, Math.toRadians(AutSample0mais4Red.HCollect3)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoOutake(2.5, robot)
        );
    }
    public Action GoToCollectSample4(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                //.splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4 - 4, Math.toRadians(AutSample0mais4Red.HCollect4)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-50, 50))
                //.splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4 - 2, Math.toRadians(AutSample0mais4Red.HCollect4 + 10)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-10, 10))
                //todo: ver a poição do 4 porque esta travando toda hr
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4, Math.toRadians(AutSample0mais4Red.HCollect4)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-80, 80))
                //.splineToSplineHeading(new Pose2d(AutSample0mais5.XCollect4 - 2,AutSample0mais5.YCollect4, Math.toRadians(AutSample0mais5.HCollect4 - 30)), Math.toRadians(tangentCollect4 - 40))
                .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoOutake(2.5, robot)
        );
    }
    public Action GoToCollectSample5(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                .waitSeconds(0.3)
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect5part2 - 3, AutSample0mais4Red.YCollect5, Math.toRadians(AutSample0mais4Red.HCollect5)), Math.toRadians(AutSample0mais4Red.TangentCollect5part2))
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect5part2 + 7, AutSample0mais4Red.YCollect4, Math.toRadians(AutSample0mais4Red.HCollect4 - 30)), Math.toRadians(tangentCollect4 - 40))
                .build();

        return new SequentialAction(
                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                move,
                esperarPeloInicioDoOutake(2.5, robot)
                //new InstantAction(() -> {robot.md.pose = new Pose2d(-20, -15, Math.toRadians(0));})
        );
    }

    /************************************\
     *   Wait To Condition
     \*************************************/
    public  Action esperarPeloInicioDoOutake(double tempoLimite, V5 robot){
        // verificar se um action já terminou para passar o estado para outra
        // por enquanto só para intake
        return new Action() {
            boolean OuttakeSuperiorFuncionou                  = false;
            boolean LinearHorizontalTaRetraido                = robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState ==LinearHorizontalStates.RETRACTED;
            boolean EstadoDosInferioresEstaInicial            =  robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.INITIAL;
            boolean SampleFicouAgarrada                       = false;
            boolean naoTemSample                              = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Não há samples no intake");
            boolean temSampleMasNaoTaBoaParaPegar             = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake() && !robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();

            boolean IntakeForcando                            = (robot.intakeInferior.intakeSuccao.sugador.getCurrent(CurrentUnit.AMPS) > AutSample0mais4Red.currMax);
            double tempoLimiteParaTerminarAAction = tempoLimite;

            double lembrarPowerSugador = IntakeSuccao.power_Sugador;
            boolean podeTerminar = false, started = false;
            ElapsedTime tempoAtual = new ElapsedTime(), tempoForcando = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("Tempo: "+ tempoAtual.time());
                telemetryPacket.put("corrente sugador", robot.intakeInferior.intakeSuccao.sugador.getCurrent(CurrentUnit.AMPS));
                if(!started){
                    tempoAtual.reset();
                    started = true;
                }
                OuttakeSuperiorFuncionou       = robot.outtakeIntakeSuperior.upperSubsystemStates == UpperSubsystemStates.OUTTAKE; // garra já abriu
                double tempoRestante           = tempoLimite - tempoAtual.time();

                if((temSampleMasNaoTaBoaParaPegar && LinearHorizontalTaRetraido  && EstadoDosInferioresEstaInicial) || (LinearHorizontalTaRetraido && EstadoDosInferioresEstaInicial && naoTemSample)){
                    if(tempoAtual.time() > 1.900){
                       // robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    }

                }
                if(IntakeForcando) {
                    if(tempoForcando.time() < 0.15){
                        IntakeSuccao.power_Sugador = 1;
                    }
                    else if(tempoForcando.time() < 0.35){
                        IntakeSuccao.power_Sugador = -0.2;
                    }else{
                        tempoForcando.reset();
                        IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    }
                }else{
                    tempoForcando.reset();
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                }

                if(OuttakeSuperiorFuncionou){
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    return  false;
                }

                podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction;
                if(IntakeForcando) podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction + 1;

                if(tempoAtual.time() >= tempoLimiteParaTerminarAAction - 1.4 && robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED && !robot.intakeInferior.linearHorizontalMotor.isBusy){
                    robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
                }
                if(tempoAtual.time() >= tempoLimiteParaTerminarAAction && IntakeForcando && !podeTerminar){
                    robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                }
                if(podeTerminar ){
                    if (sampleID < 4) robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    else robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    return false;
                }
                return true ;
            }
        };
    }
    public Action esperarPraSoltarSample(double delay, V5 robot){
        return new Action() {
            boolean podeAbrirAGarra = false, jaChegouNoTarget = false, podeTerminarAaction = false, started = false;

            double tempoPraAbrirAgarra = delay;
            double tempoPraTerminarAAction = tempoPraAbrirAgarra + 0.22;
            // aumeentar um pouquinho

            final ElapsedTime tempoQueChegou = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("COMEÇANDO A ESPERAR");
                if(!started) {
                    tempoQueChegou.reset();
                    started = true;
                }


                jaChegouNoTarget = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 2500;

                if(!jaChegouNoTarget) tempoQueChegou.reset();

                telemetryPacket.addLine("Vertical Chegou no target: "+ jaChegouNoTarget);
                telemetryPacket.addLine("TempoQueChegou: "+ tempoQueChegou.time());

                podeAbrirAGarra = tempoQueChegou.time() >= tempoPraAbrirAgarra;
                podeTerminarAaction = tempoQueChegou.time() >= tempoPraTerminarAAction;

                if(podeAbrirAGarra){
                    telemetryPacket.addLine("Garra ja abriu");
                    robot.outtakeIntakeSuperior.garraSuperior.garraOpeningState = GarraOpeningStates.OPEN;
                    robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(robot.outtakeIntakeSuperior.garraSuperior.mapOpening.get(robot.outtakeIntakeSuperior.garraSuperior.garraOpeningState));
                }
                else{
                    telemetryPacket.addLine("Garra esperando para Abrir");
                }
                if(podeTerminarAaction){
                    telemetryPacket.addLine("Garra ja abriu");
                    return false;
                }



                return true;
            }
        };
    }
    public Action esperarPeloOutakeTerSubidoEextender(double delay, V5 robot){
        return new Action() {
            boolean jaSubiu = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("Esperando pra extender");

                jaSubiu = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 1200;

                if(jaSubiu){
                    telemetryPacket.addLine("Extendendo");
                    if(sampleID < 4) {
                        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    }else{
                        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
                    }

                    return false;
                }



                return true;
            }
        };
    }

    /***************************\
     *   Verificações
     \**************************/

    public Action LimelightCalculateCorrection(double targetZ, V5 robot) {
        return new Action() {
            final ElapsedTime temporizador = new ElapsedTime();
            boolean started = false;

            double currentZ = 0, erro = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Inicia o cronômetro na primeira execução
                if (!started) {
                    temporizador.reset();
                    started = true;
                }
                currentZ = robot.limelight.getZdistanceFromAprilTag();

                // Calcula o erro de posição
                if(currentZ != 0){
                    erro = targetZ - currentZ;
                    limelightCorrection = erro * AutSample0mais4Red.LimeLightKp;
                }
                // Define um fator de correção proporcional (ajuste conforme necessário)

                // Enviar informações para telemetria
                telemetryPacket.put("Limelight Z", currentZ);
                telemetryPacket.put("Erro Z", erro);
                telemetryPacket.addLine("limelightCorrection("+limelightCorrection+" = ( targetZ("+targetZ+") - currentZ("+currentZ+") ) * kp("+ AutSample0mais4Red.LimeLightKp+")");
                telemetryPacket.put("Correção Aplicada", limelightCorrection);

                // Se o erro for pequeno o suficiente ou o tempo limite for atingido, finaliza a ação
                if ( temporizador.time() >= AutSample0mais4Red.LimelightDelayToRead) {
                    if(Math.abs(erro) < 0.02 || Math.abs(erro) > 1 || erro > 0.5) limelightCorrection = 0;
                    return false;
                }

                return true;
            }
        };
    }

}
