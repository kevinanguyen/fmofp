"""
Test suite: loop-prevention category registration (story B3.1).

TWO FINDINGS.

1. register_category DID NOT EXIST, AND FIVE CALL SITES PROBED FOR IT.
   Three handlers -- FCS, FMS and Radar -- each began their
   _register_*_loop_prevention_categories() with

       if not hasattr(self.loop_prevention, 'register_category'):
           logger.warning("Loop prevention middleware does not support
                           category registration - using default logic")
           return

   and the name existed nowhere in the package. So each logged and returned,
   leaving 21 category registrations below it as dead code. Measured on a
   live boot: that warning exactly 7 times, which is the figure the PI's own
   risk register recorded five months earlier.

   Two further call sites, in interface_display_message_handler and
   display_message_router, guard with a bare `if hasattr(...)` and no else --
   so they skipped registration SILENTLY, which is worse. 23 registrations
   dead in total across the five sites.

2. CATEGORY EXTRACTION DEPENDED ON PYTHONHASHSEED. _extract_category returned
   the first match while iterating _common_categories, which is a SET.
   Measured with the finer names registered:
   _extract_category({'message_type': 'fcs_statusResponse'}) returned
   'fcs_status' on 2 of 5 hash seeds and 'status' on the other 3 -- the same
   message, categorised differently from run to run.

   That is why finding 1 could not be fixed on its own: 16 of the 21 intended
   names are compound forms of names already in the set ('fcs_status' beside
   'status', 'fms_mode' beside 'mode_change'), so registering them would have
   multiplied the overlaps and made the coin-flip more frequent, not less.
   Resolved by longest-match with an alphabetical tie-break: deterministic,
   and the most specific answer rather than an arbitrary one.

WHAT WAS NOT IMPLEMENTED, deliberately. All five sites pass category_type,
priority and max_simultaneous_processing, and nothing in the package reads any
of them -- asserted below, not assumed. Honouring them would mean inventing a
priority scheduler and a per-category concurrency limiter, which is a feature
rather than a fix. register_category records them in _category_metadata so the
declared intent is retrievable, and says so in its docstring.

Run:  python3 -m FMOFP.Tests.test_loop_prevention_categories
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pathlib
import subprocess
import textwrap
import traceback

from FMOFP.Tests import code_has


class _Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self._failures = []

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            msg = f"  FAIL  {name}" + (f"  [{detail}]" if detail else "")
            print(msg)
            self._failures.append(msg)

    def section(self, title):
        print(f"\n{title}")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n  {self.passed}/{total} passed")
        if self._failures:
            print("\n  Failures:")
            for f in self._failures:
                print(f"    {f}")
        return self.failed == 0


R = _Results()
_FMOFP = os.path.join(_B20SS, 'FMOFP')

CALL_SITES = (
    'local_messaging/routing/handlers/system_message_handlers/FCSMessageHandler.py',
    'local_messaging/routing/handlers/system_message_handlers/FMSMessageHandler.py',
    'local_messaging/routing/handlers/system_message_handlers/RadarMessageHandler.py',
    'Interfaces/userInterface/messaging/interface_display_message_handler.py',
    'Interfaces/userInterface/messaging/display_message_router.py',
)


def _src(rel):
    return pathlib.Path(os.path.join(_FMOFP, rel)).read_text(encoding='utf-8')


def test_the_method_the_probes_look_for_exists():
    R.section("register_category exists on what the handlers probe")

    from FMOFP.Utils.message_loop_prevention.middleware import (
        MessageLoopPreventionMiddleware, get_loop_prevention_middleware)
    from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention

    # The probe is hasattr on the MIDDLEWARE, which is a different class in a
    # different module from the prevention service it wraps. Putting the
    # method only on the service would have left every probe failing.
    R.check("the middleware class has register_category",
            callable(getattr(MessageLoopPreventionMiddleware,
                             'register_category', None)))
    R.check("the singleton accessor's instance has it too",
            hasattr(get_loop_prevention_middleware(), 'register_category'))
    R.check("the prevention service it delegates to has it",
            callable(getattr(MessageLoopPrevention, 'register_category', None)))
    R.check("and the metadata getter exists on both",
            callable(getattr(MessageLoopPreventionMiddleware,
                             'get_category_metadata', None))
            and callable(getattr(MessageLoopPrevention,
                                 'get_category_metadata', None)))

    for rel in CALL_SITES:
        src = _src(rel)
        # RAW source for the probe. The thing being matched is the string
        # literal 'register_category' inside hasattr(), and code_has() strips
        # string literals -- so it can never see it. Seventh variation of
        # that trap, and the first one walked into using the very helper
        # written to avoid it. The CALL below is code, so code_has suits it.
        R.check(f"{os.path.basename(rel)} still probes for it",
                "hasattr(self.loop_prevention, 'register_category')" in src
                or 'hasattr(self.loop_prevention, "register_category")' in src,
                rel)
        R.check("...and calls it",
                code_has(src, "self.loop_prevention.register_category("), rel)


def test_registering_makes_a_category_classifiable():
    R.section("a registered name becomes something extraction can return")

    from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention

    p = MessageLoopPrevention.get_instance()
    before = set(p._common_categories)
    R.check("a freshly-invented name is not classifiable yet",
            'b31_probe_category' not in before)

    name = p.register_category('B31_Probe_Category', category_type='probe',
                               priority='highest', max_simultaneous_processing=1)
    R.check("register_category lowercases and returns the name",
            name == 'b31_probe_category', repr(name))
    R.check("...and it joins the classifiable set",
            'b31_probe_category' in p._common_categories)
    R.check("...so a message naming it is categorised as it",
            p._extract_category({'message_type': 'b31_probe_category_update'})
            == 'b31_probe_category',
            p._extract_category({'message_type': 'b31_probe_category_update'}))

    meta = p.get_category_metadata().get('b31_probe_category', {})
    R.check("the metadata it was given is recorded",
            meta.get('category_type') == 'probe'
            and meta.get('priority') == 'highest'
            and meta.get('max_simultaneous_processing') == 1, str(meta))
    R.check("...and says plainly that nothing consumes it",
            'recorded only' in meta.get('note', ''), str(meta.get('note')))


def test_nothing_consumes_the_metadata_it_records():
    R.section("NON-TAUTOLOGICAL — the docstring's claim is checked, not trusted")

    # register_category's docstring says priority, category_type and
    # max_simultaneous_processing are recorded and not acted upon. If someone
    # implements them, this fails and the docstring has to be corrected --
    # which is the point of asserting it.
    #
    # Done by AST, because a text scan cannot tell a PARAMETER DECLARATION
    # from a use: register_category's own signature names all three, and the
    # first version of this check reported those four lines as consumers.
    import ast

    FIELDS = {'priority', 'category_type', 'max_simultaneous_processing'}
    RECORDERS = {'register_category', 'get_category_metadata'}
    pkg = os.path.join(_FMOFP, 'Utils', 'message_loop_prevention')
    readers = []

    for path in sorted(pathlib.Path(pkg).glob('*.py')):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        # every function whose name is not one of the recorders
        for fn in [n for n in ast.walk(tree)
                   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and n.name not in RECORDERS]:
            declared = {a.arg for a in fn.args.args + fn.args.kwonlyargs}
            for node in ast.walk(fn):
                # a load of a local or attribute by that name
                if (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
                        and node.id in FIELDS and node.id not in declared):
                    readers.append(f"{path.name}:{node.lineno} name {node.id}")
                if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
                        and node.attr in FIELDS):
                    readers.append(f"{path.name}:{node.lineno} attr {node.attr}")
                # a dict lookup keyed by one of them
                if (isinstance(node, ast.Subscript)
                        and isinstance(node.slice, ast.Constant)
                        and node.slice.value in FIELDS):
                    readers.append(f"{path.name}:{node.lineno} subscript "
                                   f"{node.slice.value}")

    R.check("no code in the package reads priority, category_type or "
            "max_simultaneous_processing", not readers, str(readers[:4]))

    # And the scanner must be able to see a reader, or the above is free.
    planted = ast.parse(textwrap.dedent("""
        def schedule(self, settings):
            if settings['priority'] == 'highest':
                return 1
    """))
    hits = [n for n in ast.walk(planted)
            if isinstance(n, ast.Subscript) and isinstance(n.slice, ast.Constant)
            and n.slice.value in FIELDS]
    R.check("the scanner flags a planted consumer", len(hits) == 1, str(hits))


def _extract_under_seed(seed, names, message_type):
    script = textwrap.dedent(f'''
        import sys
        sys.path.insert(0, {_B20SS!r})
        sys.path.insert(0, {os.path.join(_B20SS, 'FMOFP')!r})
        from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention
        p = MessageLoopPrevention.get_instance()
        for n in {names!r}:
            p.register_category(n)
        print("CAT:" + str(p._extract_category({{'message_type': {message_type!r}}})))
    ''')
    env = dict(os.environ)
    env['PYTHONHASHSEED'] = str(seed)
    env['QT_QPA_PLATFORM'] = 'offscreen'
    env['PYTHONUTF8'] = '1'
    env['PYTHONIOENCODING'] = 'utf-8'
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True,
                          text=True, timeout=180, cwd=_B20SS, env=env)
    for line in (proc.stdout or '').splitlines():
        if line.startswith('CAT:'):
            return line[4:]
    return f'<no result: {(proc.stderr or "")[-120:]}>'


def test_extraction_is_the_same_under_every_hash_seed():
    R.section("category extraction no longer depends on PYTHONHASHSEED")

    names = ['fcs_status', 'fcs_mode', 'fms_status', 'weather_radar_status']
    answers = {_extract_under_seed(s, names, 'fcs_statusResponse')
               for s in (1, 2, 3, 4, 5)}
    R.check("five hash seeds give one answer", len(answers) == 1, str(answers))
    R.check("...and it is the most specific match, not the coarse one",
            answers == {'fcs_status'}, str(answers))

    # NON-TAUTOLOGICAL: the old rule, reproduced inline, really was
    # order-dependent -- so the check above is not passing for free.
    import random
    from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention
    p = MessageLoopPrevention.get_instance()
    for n in names:
        p.register_category(n)
    haystack = 'fcs_statusresponse'
    candidates = [c for c in p._common_categories if c in haystack]
    R.check("more than one registered category matches that message type",
            len(candidates) > 1, str(sorted(candidates)))

    first_match_answers = set()
    for _ in range(40):
        shuffled = list(p._common_categories)
        random.shuffle(shuffled)
        for c in shuffled:                      # the old rule
            if c in haystack:
                first_match_answers.add(c)
                break
    R.check("...and first-match-over-an-unordered-set gives several answers",
            len(first_match_answers) > 1, str(sorted(first_match_answers)))
    R.check("...while longest-match gives exactly one",
            p._best_category_match(haystack) == 'fcs_status',
            p._best_category_match(haystack))


def test_registration_does_not_let_anything_bypass_prevention():
    R.section("the gate is unchanged: no category now skips detection")

    # process_message returns early when a category is disabled, so a new
    # category that resolved to disabled would silently switch loop
    # prevention OFF for those messages. Unknown categories default to
    # enabled, which is why this holds -- asserted because the consequence
    # of being wrong is invisible.
    from FMOFP.Utils.message_loop_prevention.config import get_config
    from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention

    cfg = get_config()
    p = MessageLoopPrevention.get_instance()
    for n in ('fcs_mode', 'fms_command', 'weather_radar_status', 'display_data'):
        p.register_category(n)
    disabled = [n for n in p.get_category_metadata()
                if not cfg.is_category_enabled(n)]
    R.check("every registered category is still enabled", not disabled,
            str(disabled))
    R.check("an unknown category is enabled by default, which is what makes "
            "that safe", cfg.is_category_enabled('a_category_nobody_registered'))


def main():
    print("=" * 60)
    print("  Loop-prevention category registration (B3.1)")
    print("=" * 60)

    for test in (test_the_method_the_probes_look_for_exists,
                 test_registering_makes_a_category_classifiable,
                 test_nothing_consumes_the_metadata_it_records,
                 test_extraction_is_the_same_under_every_hash_seed,
                 test_registration_does_not_let_anything_bypass_prevention):
        try:
            test()
        except Exception:
            R.failed += 1
            R._failures.append(f"  FAIL  {test.__name__} raised")
            print(f"  FAIL  {test.__name__} raised:")
            traceback.print_exc()

    print("\n" + "=" * 60)
    ok = R.summary()
    print("=" * 60)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
