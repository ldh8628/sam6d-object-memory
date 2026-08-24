# Blind Hunter prompt

Use the `bmad-review-adversarial-general` skill. You receive only the content of
`pem-independent-review-diff.md`; do not inspect the spec, conversation, context docs, or
project files. Treat the NO_VCS manifest as the best-effort diff and find at least ten
precise problems or missing safeguards. Return only a Markdown list of finding descriptions.
