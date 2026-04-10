package frc.robot.diagnostics;

/*
 * Interface to perfrom a system check for all subsystem, to be called in pits.
 */
public interface SystemCheck {
    public DiagnosticResult performSystemCheck();
}
