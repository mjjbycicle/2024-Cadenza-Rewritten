package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class CommandFactory {
    private static final ShooterSuperstructure shooter = ShooterSuperstructure.getInstance();
    private static final IndexerSuperstructure indexer = IndexerSuperstructure.getInstance();

    public static Command autoShootCommand() {
        return Commands.sequence(
                        shooter.setShootingMode(ShootingMode.AUTO),
                        Commands.sequence(
                                Commands.waitUntil(
                                        shooter.atRequestedStateTrigger()
                                ),
                                indexer.setIndexerStateCommand(IndexerState.FEED)
                        ),
                        Commands.waitUntil(indexer.hopperHasNote().negate()),
                        Commands.parallel(
                                indexer.setIndexerStateCommand(IndexerState.IDLE),
                                shooter.setShootingMode(ShootingMode.IDLE)
                        )
                );
    }
}
