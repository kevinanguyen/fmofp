"""Test package bootstrap.

Exists for one reason: force UTF-8 on stdout and stderr for every suite.

The suites print box-drawing separators and check/cross marks (7,452 non-ASCII
characters across 30 files, 7,222 of them U+2500 alone). When a suite's stdout
is a PIPE -- which it always is under run_all_tests.py, and often is when a
developer redirects output -- Python picks the locale encoding for it, and on
Windows that is cp1252. The first such print() then raises UnicodeEncodeError
before the suite has asserted anything, and the handler raises again trying to
print the failure marker. Nine of the 28 suites died that way.

run_all_tests.py sets PYTHONIOENCODING for the children it launches, which
fixes runs through the runner. This covers every other invocation -- notably
`python -m FMOFP.Tests.<suite>` run directly -- because importing any
FMOFP.Tests submodule imports this first.

reconfigure() overrides PYTHONIOENCODING, so it is authoritative wherever it
can run at all; a closed or exotic stream simply keeps its current encoding.
"""

import sys


def force_utf8_stdio():
    """Best effort: make stdout/stderr encode the glyphs the suites print."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError, ValueError):
            pass


force_utf8_stdio()


def code_only(source: str) -> str:
    """Strip comments and string literals from Python source.

    For source-level assertions of the form "is this still CALLED". A defect
    is almost always NAMED in the comment that explains why it was fixed, so a
    plain substring check against raw source reports the fix as absent and
    fails on correct code. That mistake was made four separate times in this
    test suite before this helper was shared:

      * a comment-stripping string replace that did nothing;
      * a 600-character window after a `def` that stopped inside the docstring;
      * an ordering check that matched `connect(` in a comment above the line;
      * and, in the other direction, a check for "{request.retry_count}" that
        failed because STRINGS are stripped too.

    That last one is the limit worth remembering: f-strings go as well, so this
    can say nothing about logged text. Check raw source for that.

    A fifth mistake, and the reason code_has() exists below: the result is
    tokens joined by SPACES, so `self.log_level` comes back as
    `self . log_level` and a natural-looking substring check for
    "file_handler.setLevel(self.log_level)" finds nothing in correct source.
    Match with code_has(), which ignores whitespace on both sides, rather than
    spelling the spaces out by hand.

    Exercise it both ways before trusting a result -- it should flag a planted
    call and spare a commented mention.
    """
    import io
    import tokenize

    kept = []
    try:
        for tok in tokenize.generate_tokens(io.StringIO(source).readline):
            if tok.type in (tokenize.COMMENT, tokenize.STRING):
                continue
            kept.append(tok.string)
    except (tokenize.TokenError, IndentationError):
        # Fall back to a line filter rather than claiming a clean result.
        return "\n".join(l for l in source.splitlines()
                          if not l.strip().startswith("#"))
    return " ".join(kept)


def code_has(source: str, needle: str) -> bool:
    """True when `needle` appears in `source` as CODE, ignoring whitespace.

    Wraps code_only() and strips whitespace from both sides, so a needle can
    be written the way the code reads -- "file_handler.setLevel(self.log_level)"
    -- rather than the way code_only() emits it, token by token with spaces
    between. Comments and string literals are still invisible, with the same
    consequence: this cannot see logged text.
    """
    compact = "".join(code_only(source).split())
    return "".join(needle.split()) in compact
