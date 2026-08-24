# Edge Case Hunter prompt

Use the `bmad-review-edge-case-hunter` skill. Read
`implementation-artifacts/pem-independent-review-diff.md`, then inspect only the changed
files listed in its review scope and their directly referenced helpers. Exhaustively trace
branches and boundaries introduced by the change. Return only the skill's exact JSON-array
format; report unhandled reachable cases, not style opinions.
