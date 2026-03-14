# Operator Controller Diagnosis TODO

## Plan Steps:
- [x] 1. Add telemetry/logging for controller1 (operator, port 1) inputs in RobotContainer.java periodic(): axes (leftY/X, rightX), triggers, buttons A/B/rightTrigger.
- [ ] 2. Deploy to sim/real robot (e.g., ./gradlew simulateJava or deploy).
- [ ] 3. Check DriverStation controller list (ensure operator controller on USB port 1, enabled).
- [ ] 4. Observe SmartDashboard values (Driver/Operator/*): Press operator sticks/buttons, see if change from 0.
- [ ] 5. Test bindings: Press A/B (intake deploy/raise), rightTrigger (intake roller).
- [ ] 6. Report back: Logs change? Bindings work? If not, hardware/DS issue.

Current progress: Step 1 complete. Proceed to test.
