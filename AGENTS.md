# Agent instructions

## Workplans

Triggered when I say "use a workplan".

1. Create `WORKPLAN_<feature>.md` in the repo root. Write no code yet.
2. Fill it in using the format below, show me the todo list, and stop. Wait for my approval.
3. Once approved, implement top to bottom. Tick each item off in the file the moment it is done — not in batches at the end.
4. Record decisions in the file as you make them, not at the end. A decision you have not written down is a decision you will lose.
5. If you discover work that is not in the plan, add it as a todo rather than doing it silently.
6. I'll take care of the file once the feature is done.

You own exactly one workplan: the one for the feature I asked you to implement. Any other `WORKPLAN_*.md` belongs to another agent working in parallel — never read, edit, or delete it.

If you are unsure what is left, re-read your workplan. It is the source of truth, not your recollection.

Workplans are not committed.

### Format

Use these headings exactly, in this order. Tooling depends on it.

```markdown
# <feature>

Goal: <one or two sentences>

## Todos
- [ ] ...
- [ ] ...

## Decisions
- <what was chosen, what was rejected, and why>
- <approaches that failed, so they are not retried>
```
