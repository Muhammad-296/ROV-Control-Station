# Continuous integration guide

The `.github/workflows` directory contains two independent checks. Both run on pull requests and can be started manually through **Actions → Run workflow**. Push triggers are scoped to relevant files on `main`.

## Python checks

The `python-checks.yml` workflow installs Python 3.12, runs `py_compile` on the GUI and test script, then runs `test_protocol.py`. These tests intentionally avoid a real GUI window and external motors. They do **not** install heavyweight Qt/VTK packages, render the dashboard or test live camera/PS4 input.

## Arduino Mega compilation

The `arduino-compile.yml` workflow installs Arduino CLI, the official AVR core, OneWire and DallasTemperature, then compiles the firmware sketch using `arduino:avr:mega`. Compilation verifies firmware dependencies and basic AVR compatibility; it does **not** upload to an Arduino or validate actual wiring, safety, timing under load or underwater operation.

## Recommended merge criteria

1. Python syntax + protocol checks pass.
2. Arduino Mega compile passes.
3. Any changes affecting motor output, safety state or pin mapping have an operator-reviewed physical test plan and appropriate recorded validation.
4. No personal credentials or generated builds are committed.

Status badges appear on the README and will reflect actual GitHub-hosted workflow results after the first run. If the official Arduino CLI action or library installation changes upstream, update the workflow and check the workflow logs rather than assuming successful compilation.
