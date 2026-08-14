---
name: resolving-merge-conflicts
description: "Use when you need to resolve an in-progress git merge/rebase conflict."
---

1. **See the current state** of the merge/rebase. Check git history, and the conflicting files.

2. **Find the primary sources** for each conflict. Understand deeply why each change was made, and what the original intent was. Read the commit messages, check the PRs, check original issues/tickets.

3. **Resolve each hunk.** Preserve both intents where possible. Where incompatible, pick the one matching the merge's stated goal and note the trade-off. Do **not** invent new behaviour. Do not abort, stage, or continue the operation automatically.

4. Discover the project's **automated checks** and run them. For UNIMOC,
   use the `hosted-build-test` skill for hosted builds, tests, clang-tidy, and
   clang-format. Fix anything the merge broke.

5. **Hand back the lifecycle boundary.** Leave resolved files unstaged and
	report whether the repository is in a merge or rebase state. The user can
	stage, commit, abort, or continue it explicitly.
