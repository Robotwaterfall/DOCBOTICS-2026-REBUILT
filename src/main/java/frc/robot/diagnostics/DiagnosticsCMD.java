package frc.robot.diagnostics;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * Command that runs all of the diagnostics for all subsystems
 */
public class DiagnosticsCMD extends Command {
    private final List<SubsystemBase> subsystems;

    // Constructor
    public DiagnosticsCMD(SubsystemBase... subsystems) {
        this.subsystems = List.of(subsystems);
    }

    /**
     * Description: Runs all of the diagnostics
     * Pre-Condition: None
     * Post-Condition: All subsystems that are able to be diagnosed are diagnosed
     */
    @Override
    public void initialize() {
        for (SubsystemBase s : subsystems) {

            // Only run diagnostics if the subsystem implements Diagnosable
            if (s instanceof Diagnosable d) {

                DiagnosticResult result = d.runDiagnostics();

                SmartDashboard.putBoolean(result.getName() + " OK", result.allPassed());

                result.getChecks().forEach((label, passed) -> {
                    SmartDashboard.putBoolean(result.getName() + " - " + label, passed);
                });
            }
        }
    }

    // Finishes the method, there is no excecute since this command only runs once
    @Override
    public boolean isFinished() {
        return true;
    }

}

