package frc.robot.diagnostics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/*
 * A helper class for diagnostics. Is able to contain information about diagnostics for each subsystem
 */
public class DiagnosticResult {
    
    // Atribuites
    private final String name;
    private final Map<String, Boolean> checks = new HashMap<>();
    private final ArrayList<DiagnosticResult> subResults = new ArrayList<DiagnosticResult>();

    // Constructor
    public DiagnosticResult(String name) {
        this.name = name;
    }

    /**
     * Description: Performs a check on the given label to see if the boolean passed is true or false
     * Pre-Condition: None
     * Post-Condition: The check is added to the list of checks
     * @param label The label that is being checked
     * @param passed The actual thing being checked
     */
    public void check(String label, boolean passed) {
        checks.put(label, passed);
    }

    /**
     * Description: Performs a check on the given lable to see if it is working as intended above the pass threshold
     * Pre-Condition: None
     * Post-Condition: The check is added to the list of checks
     * @param label The label that is being checked
     * @param test The test that is being tested
     * @param iterations The number of times to be tested (Usually 10)
     * @param passThreshold The ratio of passes to iterations required for test to be successful (Usually 0.8)
     */
    public void checkRepeated(String label, BooleanSupplier test, int iterations, double passThreshold) {
        int passes = 0;

        for (int i = 0; i < iterations; i++) {
            if (test.getAsBoolean()) {
                passes++;
            }
        }

        boolean passed = ((double) passes / iterations) >= passThreshold;
        checks.put(label, passed);
    }

    /**
     * Description: Checks if every diagnostic check passed
     * Pre-Condition: Checks is not empty
     * Post-Condition: A boolean is returned based on the successes or failure of each trial
     * @return True if each test passed, false if there was a failure
     */
    public boolean allPassed() {
        boolean selfPassed = checks.values().stream().allMatch(b -> b);
        boolean childrenPassed = subResults.stream().allMatch(DiagnosticResult::allPassed);
        return selfPassed && childrenPassed;
    }

    /**
     * Description: Adds a subResult to this diagnostic result
     * Pre-Condition: Param is a DiagnosticResult
     * Post-Condition: a subResult is added to this diagnostic result
     * @param child The diagnostic result being added as a subResult
     */
    public void addSubResult(DiagnosticResult child) {
        subResults.add(child);
    }

    // Getters
    public Map<String, Boolean> getChecks() {
        return checks;
    }

    public List<DiagnosticResult> getSubResults() {
        return subResults;
    }

    public String getName() {
        return name;    
    }

}
