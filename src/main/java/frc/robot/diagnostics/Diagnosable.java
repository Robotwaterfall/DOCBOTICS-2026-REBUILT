package frc.robot.diagnostics;

/*
 * Interface for a subsystem to implement that shows it is able to run diagnostics
 */
public interface Diagnosable {
    DiagnosticResult runDiagnostics();
}
