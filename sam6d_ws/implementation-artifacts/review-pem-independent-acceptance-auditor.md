# Acceptance Auditor prompt

Read `implementation-artifacts/pem-independent-review-diff.md`,
`implementation-artifacts/spec-pem-size-iou-candidate-filter.md`, and both context specs in
its frontmatter. Inspect the changed project files. Audit every frozen boundary, I/O row,
task and acceptance criterion against code, tests and generated outputs. Pay special
attention to geometry-only selection invariance, absence of hidden combined-score ranking,
independent verification semantics, legacy report compatibility, existing-index safe
publication, proxy provenance and unsupported GT/accuracy claims. Return a concise Markdown
finding list with severity, evidence path:line, and the violated requirement; return `[]`
only if no violation exists.
