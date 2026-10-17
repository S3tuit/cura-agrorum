# Agent instructions

## Documentation

- Explain what is difficult to infer: design reasons, invariants, surprising
  constraints and failure consequences. Do not paraphrase self-explanatory code
  or script behavior.
- Put local behavior explanations beside the owning function, class or module.
  Keep them concise; do not move whole specifications into giant docstrings.
- Give each fact one authoritative home. Link to schemas, configuration, code
  and existing procedures instead of copying field lists, defaults or behavior.
- Create documents for a concrete reader and task, such as navigation, setup,
  hardware assembly or operator maintenance. A directory does not automatically
  need a README. Delete documents that no longer serve a reader or tool.
- Describe the current system and the reasons for its choices. Preserve useful
  failure lessons; remove implementation chronology, completed plans and
  "new versus old" narratives.
- Keep session material in ignored `.notes.md` files and temporary test outputs.
  Never make them necessary to build, test or understand the system.
- Keep deployment records in `../cura-agrorum-logbook/`, not here.
- Keep lasting lessons beside the relevant code, regression test or procedure.
  Do not create permanent pass ledgers.
- Update or remove affected documentation and repair references in the same
  change as its owner. Ask of each paragraph: what decision or task does this
  help with that the nearby code does not already make clear?
- Documentation grows by default, so push against it. Prefer rewriting an
  existing paragraph over appending a new one. When a document keeps growing,
  check whether it still serves one reader and task.

## Workplans

Triggered when I say "use a workplan".

1. Choose a stable lowercase `snake_case` workload ID and create `WORKPLAN_<workload_id>.md` in the repo root. Write no code yet.
2. Fill it in using the format below, show me the todo list, and stop. Wait for my approval.
3. Assign stable sequential IDs when the plan is created: `T-001`, `T-002`, ... for todos and `D-001`, `D-002`, ... for decisions. Never renumber or reuse an ID.
4. Once approved, implement top to bottom. Tick each item off in the file the moment it is done — not in batches at the end.
5. Record decisions in the file as you make them, not at the end. A decision you have not written down is a decision you will lose.
6. If you discover work that is not in the plan, add it with the next unused todo ID rather than doing it silently. Give each new decision the next unused decision ID.

Do not assume the smallest correct patch is the best design. If a local fix is available but a broader redesign is also credible and would materially simplify the system, reduce coupling, remove special cases, or improve the abstraction, present both approaches, their costs, and your recommendation. Ask for my direction and ideas before implementing either approach in that area.

You own exactly one workplan: the one for the feature I asked you to implement. Any other `WORKPLAN_*.md` belongs to another agent working in parallel — never read, edit, or delete it.

If you are unsure what is left, re-read your workplan. It is the source of truth, not your recollection.

Workplans are not committed.

### Format

Use these headings exactly, in this order. Tooling depends on it.

```markdown
# <feature>

Workload-ID: <lowercase_snake_case_id>

Goal: <one or two sentences>

## Todos
- [ ] T-001: ...
- [ ] T-002: ...

## Decisions
- D-001: <what was chosen, what was rejected, and why>
- D-002: <approaches that failed, so they are not retried>
```
