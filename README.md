# FLUX Robotics 2025 Code

[![CI](https://github.com/FRC-Flux-Robotics/test-robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC-Flux-Robotics/test-robot/actions/workflows/ci.yml)

Code for robots in 2025 season:
- Coral Elevator robot
- Algae Intake robot

## Code Coverage

Coverage reports are generated on every CI run:

- **Job Summary**: Coverage percentage is displayed in the GitHub Actions job summary
- **HTML Report**: Download the `coverage-report` artifact from any CI run for detailed coverage
- **Minimum**: 30% instruction coverage enforced by JaCoCo

To generate coverage locally:
```bash
./gradlew check
open build/reports/jacoco/test/html/index.html
```
