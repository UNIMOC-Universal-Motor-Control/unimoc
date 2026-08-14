---
name: implement
description: "Implement a piece of work based on a spec or set of tickets."
disable-model-invocation: true
---

Implement the work described by the user in the spec or tickets.

Use /tdd where possible, at pre-agreed seams.

For this repository, use the `hosted-build-test` skill for focused and full
hosted validation. Run focused tests regularly and the full hosted workflow
once at the end.

Once done, use /code-review to review the work.

Do not stage or commit the work automatically. Report the changes and
validation results; only create a commit when the user explicitly requests it.
