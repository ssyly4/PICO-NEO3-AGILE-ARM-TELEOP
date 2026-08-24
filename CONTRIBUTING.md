# Contributing

Keep hardware-specific values in `.env`. Do not commit datasets, logs, APKs,
credentials, absolute home paths, or generated Unity directories. New robot
motion must have a non-executing preflight and an explicit `--execute` gate.
Do not commit the PICO Unity OpenXR SDK under `pico_client/LocalPackages/`.

Before a pull request, run Python compilation, unit tests, `bash -n` on shell
launchers, and `git diff --check`. State whether physical validation was run.
