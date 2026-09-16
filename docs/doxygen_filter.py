#!/usr/bin/env python3
"""Doxygen INPUT_FILTER: make the headers' Markdown links resolve.

Headers link to design documents with paths that are correct for GitHub and
editors, e.g. `[ADR-0006](../../../../docs/adr/0006-....md)`. Doxygen resolves
a Markdown link to another input file by matching the tail of its path, and a
leading `../` chain defeats that. Strip the chain so the link becomes
`docs/adr/0006-....md`, which Doxygen turns into a link to the rendered page.
The source files are not modified.
"""
import re
import sys

UP = re.compile(r"\]\((?:\.\./)+")

with open(sys.argv[1], encoding="utf-8") as f:
    sys.stdout.write(UP.sub("](", f.read()))
