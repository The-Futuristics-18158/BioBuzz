package org.firstinspires.ftc.teamcode.Commands.Utility;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;

/**
 * Three-way branching conditional command utility for command-based sequences.
 * Evaluates conditions sequentially and executes the first matching command, or a default fallback command.
 *
 * @author knutt5
 */
public class ThreeWayConditionalCommand extends CommandBase {

    private final Command m_Command1;
    private final Command m_Command2;
    private final Command m_CommandElse;
    private final BooleanSupplier m_Condition1;
    private final BooleanSupplier m_Condition2;
    private Command m_selectedCommand;

    /**
     * Creates a three-way conditional command to select among commands based on suppliers.
     *
     * @author knutt5
     *
     * @param condition1 boolean supplier condition to evaluate for command 1
     * @param command1 command to run if condition 1 returns true
     * @param condition2 boolean supplier condition to evaluate for command 2
     * @param command2 command to run if condition 2 returns true
     * @param commandElse default fallback command to run if no conditions return true
     */
    public ThreeWayConditionalCommand(BooleanSupplier condition1,
                                      Command command1,
                                      BooleanSupplier condition2,
                                      Command command2,
                                      Command commandElse) {
        m_Command1 = command1;
        m_Command2 = command2;
        m_CommandElse = commandElse;
        m_Condition1 = condition1;
        m_Condition2 = condition2;
    }

    /**
     * Evaluates conditions, selects the appropriate command, acquires subsystem requirements, and initializes it.
     *
     * @author knutt5
     */
    @Override
    public void initialize() {
        // Evaluate conditions in sequential priority
        if (m_Condition1 != null && m_Condition1.getAsBoolean()) {
            m_selectedCommand = m_Command1;
        } else if (m_Condition2 != null && m_Condition2.getAsBoolean()) {
            m_selectedCommand = m_Command2;
        } else {
            m_selectedCommand = m_CommandElse;
        }

        // Add requirements of selected branch
        if (m_selectedCommand != null) {
            m_requirements.addAll(m_selectedCommand.getRequirements());
            m_selectedCommand.initialize();
        }
    }

    /**
     * Periodically executes the selected command.
     *
     * @author knutt5
     */
    @Override
    public void execute() {
        if (m_selectedCommand != null) {
            m_selectedCommand.execute();
        }
    }

    /**
     * Checks if the selected command has finished.
     *
     * @author knutt5
     *
     * @return true if selected command is finished or null, false otherwise
     */
    @Override
    public boolean isFinished() {
        if (m_selectedCommand != null) {
            return m_selectedCommand.isFinished();
        } else {
            return true;
        }
    }

    /**
     * Ends the selected command when finished or interrupted.
     *
     * @author knutt5
     *
     * @param interrupted true if interrupted by the scheduler
     */
    @Override
    public void end(boolean interrupted) {
        if (m_selectedCommand != null) {
            m_selectedCommand.end(interrupted);
        }
    }
}