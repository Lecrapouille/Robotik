---
name: robotik-docs
description: Edit or extend the RobotIK documentation in doc/ (tutorials, guides, concepts, reference, case-studies; README map) and the Doxygen comments of the headers. Use when documenting a new feature, plugin, assertion, API change, or when asked to fix, rewrite or review docs.
---

# RobotIK documentation

Audience: people with no robotics background, often French. The goal: pages
that read as if an experienced developer explained what they understood,
not as if a tool described a repository. Few good pages beat many thin ones.

## Map

The pages are sorted by kind (Diátaxis), as in Compages. Each folder has a
`README.md` that lists its pages in reading order; a page starts with its
`# Title`, then a back link `[⏫ Folder](README.md)`.

| Folder | Kind | A page… |
|--------|------|---------|
| doc/README.md | map | the five folders, reading order, quick paths |
| doc/tutorials/ | tutorial | teaches by doing, in order (`NN-name.md`), ends with `Next:` |
| doc/guides/ | guide | one task now: goal → steps → code → result |
| doc/concepts/ | concept | why: the problem, the idea, the choice and what it costs |
| doc/reference/ | reference | tables of facts, no story: files, plugin, planner, API, glossary |
| doc/case-studies/ | case study | one demo plugin from end to end |

A page does one job. When a new text mixes kinds, split it: the steps go in
a tutorial or a guide, the why in a concept, the keys in the reference, and
the pages link to each other. Names are lower case with dashes
(`write-a-skill.md`); a tutorial keeps its number.

Put a fact in one place and link to it
(`[assertions](../reference/scenario-file.md#assertions)`). Root README.md
has a documentation table: keep it in sync when a page is added or renamed.
Code comments point to pages by path (`doc/reference/scenario-file.md`):
update them when a page moves
(`grep -rn "doc/" include src plugins .claude AGENTS.md README.md`).

## Writing rules (from ASD-STE100, kept for French readers)

- **One word, one meaning; one meaning, one word.** doc/reference/glossary.md is the
  dictionary: a skill is never a "task", an assertion never a "check".
  Define a word once and link to it.
- **Short sentences**: 20 words at most in a procedure, 25 in an
  explanation. One idea per sentence.
- **One instruction per sentence**, imperative, the condition first: "If
  the run fails, read the report."
- **Active voice** in procedures: "The scheduler cancels the skill", not
  "the skill is cancelled".
- **No idioms, no phrasal verbs**: "remove", not "take out"; "start", not
  "kick off".
- **Three nouns in a row at most**: "the delay before the executor plans
  again", not "the plan executor replan delay".
- **Keep the articles** (the, a): the telegraphic style reads as machine
  text. (AGENTS.md and the skills stay telegraphic: their readers are agents.)
- **One topic per paragraph.**
- **The warning before the step** it is about, never after.

Not kept from ASD-STE100: the restricted dictionary. It makes explanations
dry and repetitive.

## Signs of machine writing (remove them)

- Words: delve, tapestry, testament, pivotal, crucial, vital, key (as an
  adjective), underscore, highlight, showcase, leverage, robust (except its
  technical sense), seamless, powerful, comprehensive, intricate, landscape,
  enhance, foster, "serves as", "stands as", "offers", simply, just, easy.
- Patterns: "not X but Y"; groups of three for rhythm; run-ups ("Here is the
  thing"); closers that repeat the point; a first sentence that repeats the
  heading; sentences about the page itself ("This section explores…");
  "-ing" tails ("…, ensuring that…"); stacked hedges; absolute claims.
- Layout: no signpost headings ("Key takeaways"); bold only for the one word
  a reader must not miss; no "**Label:** text" lists; prose first, a list
  only for a real enumeration, a table only for a real comparison.
- Fix by rewriting the paragraph around its point, not by swapping words.

## Doxygen comments

A comment says what the code cannot say: repeating the name of the
function is not a comment. A public class or function answers the
questions that matter for it among:

| Question | The comment says |
|----------|------------------|
| Why? | which problem it solves |
| When? | in which situation to use it (and when not to) |
| How? | its observable behaviour, or the mechanism worth knowing |
| Guarantees? | invariants, side effects, who owns what (`@par Contract`: Promises / Expects / Errors) |
| Limits? | what it does not do, special cases |
| Why this choice? | why this API exists rather than another one |

Every public class has an `@code` example of its common use, taken from a
test or a demo that compiles. Short getters keep a one-line `///`. Keep the
RobotIK syntax (`///`, `@param`, `@return`, `@ref`), the same plain English
as the pages, and the file banner: `@file` (what it is for, in everyday
words) then `@par Contract`.

## Checks before a commit

- Every tutorial step and every quoted output is run for real
  (`./build/linux-gcc-debug/bin/Robotik-Headless ... --trace`) and pasted, not
  invented. Experiment on the `build/linux-gcc-debug/lib/robotik/plugins/...`
  copy; `make <target>` restores it.
- Code excerpts come from real files (prefer HelloWorld and
  tests/Examples/TestLayerExamples.cpp); keep them in sync when the code
  changes. Never invent a call: check the header.
- Every relative link resolves: `make check-links`.
- Fresh reader pass: reread each new page as a C++ developer who opened the
  repository an hour ago. Is the first paragraph a problem they have? Is
  every robotics word defined or linked? Can they run what the page shows?
- Do not document what does not exist; mark future work as such.
