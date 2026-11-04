"""
Test suite: the radar response services' data_handler slots.

THE FINDING. RadarMessageHandler.set_async_handler built the three weather
response services correctly and then, 59 lines further down, overwrote the
handler each one had built:

    if self.precipitation_service:
        self.precipitation_service.data_handler = self.async_handler.radar_db
    if self.vil_service:
        self.vil_service.data_handler = self.async_handler.radar_db
    if self.echo_top_service:
        self.echo_top_service.data_handler = self.async_handler.radar_db

A SystemDatabase went into a slot that must hold a PrecipitationDataHandler,
a VILDataHandler or an EchoTopDataHandler. The two types overlap on exactly
one method -- store_precipitation_data -- and SystemDatabase also happens to
have store_vil_data, which is why the system looked alive and why the weather
suite kept passing.

It was not alive. _store_precipitation_data gates on
self.data_handler._verify_table_exists() before it stores anything, and
SystemDatabase has no such method, so every single store raised
AttributeError inside the try and was swallowed by the except. Measured on a
live boot with three precipitation requests, with the row count reset first:

    data_handler types   before: SystemDatabase x3
                          after: PrecipitationDataHandler, VILDataHandler,
                                 EchoTopDataHandler
    [PRECIP_STORE] errors  before: 14    after: 0
    rows in precipitation_data
                          before: 0     after: 14

Zero rows. The defect was not log noise, it was total loss of the
precipitation data the pipeline had already decoded, framed, routed and
reassembled. It also accounted for 28 of the 67 ERROR lines on a clean boot
(14 errors plus their 14 tracebacks).

WHY THE FIRST PROBE OF THIS MISLED ME, recorded because the trap is in the
code and not in the probe. get_radar_message_handler() is named like a
singleton accessor but is `return RadarMessageHandler()` -- a fresh instance
every call, with precipitation_service still None. Reading the services off
one tells you nothing about the live binding; the response services are the
real singletons and are what this suite and the probe read. Asserted below.

THE FIX is not "delete the three lines". A slot can legitimately be empty:
the services skip their own __init__ body when radar_db is None at first
construction, and PrecipitationResponseService is a singleton, so whoever
constructs it first decides. So set_async_handler now calls
_ensure_data_handler, which leaves a correct handler alone, replaces a
wrong-typed one, fills an empty slot, and -- when there is no radar_db to
build from -- clears the slot rather than leaving a wrong object in it, which
hands the work to the lazy re-initialisation each service already has on its
store path.

Run:  python3 -m FMOFP.Tests.test_radar_data_handler_binding
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ast
import pathlib
import textwrap
import traceback


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

_RADAR_HANDLER = 'local_messaging/routing/handlers/system_message_handlers/RadarMessageHandler.py'
_PRECIP_SERVICE = 'local_messaging/routing/response_services/data_response_services/precipitation_response_service.py'


def _tree(rel):
    return ast.parse(pathlib.Path(os.path.join(_FMOFP, rel)).read_text(encoding='utf-8'))


def _func(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    return None


def _radar_db_into_data_handler(scope):
    """Assignments of a `*.radar_db` value into a `*.data_handler` target.

    This is the shape of the defect, matched structurally rather than by
    text, so reformatting the line or renaming the local cannot hide it.
    Returns a list of (lineno, target, value) descriptions.
    """
    found = []
    for node in ast.walk(scope):
        if not isinstance(node, ast.Assign):
            continue
        if not (isinstance(node.value, ast.Attribute) and node.value.attr == 'radar_db'):
            continue
        for target in node.targets:
            if isinstance(target, ast.Attribute) and target.attr == 'data_handler':
                found.append((node.lineno,
                              ast.unparse(target), ast.unparse(node.value)))
    return found


# --------------------------------------------------------------------------
# 1. The detector finds the pre-fix code. Without this the guard below is
#    worth nothing -- a matcher that finds nothing anywhere also "passes".
# --------------------------------------------------------------------------

def test_the_detector_catches_the_shape_it_is_looking_for():
    R.section("the structural detector flags the pre-fix code")

    prefix = textwrap.dedent('''
        def set_async_handler(self, async_handler):
            if self.precipitation_service:
                self.precipitation_service.data_handler = self.async_handler.radar_db
            if self.vil_service:
                self.vil_service.data_handler = self.async_handler.radar_db
            if self.echo_top_service:
                self.echo_top_service.data_handler = self.async_handler.radar_db
    ''')
    hits = _radar_db_into_data_handler(ast.parse(prefix))
    R.check("flags all three pre-fix assignments", len(hits) == 3,
            f"found {len(hits)}")

    # It must not fire on the legitimate shapes: building a handler FROM the
    # db, or reading radar_db off a handler.
    benign = textwrap.dedent('''
        def f(self, radar_db):
            self.precipitation_service.data_handler = PrecipitationDataHandler(radar_db)
            db = self.data_handler.radar_db
            self.data_handler = None
    ''')
    R.check("does not fire on building a handler from the db, or on reads",
            _radar_db_into_data_handler(ast.parse(benign)) == [],
            str(_radar_db_into_data_handler(ast.parse(benign))))


# --------------------------------------------------------------------------
# 2. The defect is gone from the live file, and the call to the replacement
#    is actually there -- deleting the lines without the ensure step would
#    leave a different bug (empty slots on a re-bind).
# --------------------------------------------------------------------------

def test_set_async_handler_no_longer_assigns_the_db_into_the_slot():
    R.section("set_async_handler no longer puts a database in the slot")

    tree = _tree(_RADAR_HANDLER)
    fn = _func(tree, 'set_async_handler')
    R.check("set_async_handler is still there to check", fn is not None)
    if fn is None:
        return

    hits = _radar_db_into_data_handler(fn)
    R.check("no `.data_handler = ....radar_db` assignment in it",
            hits == [],
            "; ".join(f"line {ln}: {t} = {v}" for ln, t, v in hits))

    # and nowhere else in the module either
    whole = _radar_db_into_data_handler(tree)
    R.check("none anywhere in RadarMessageHandler.py", whole == [],
            "; ".join(f"line {ln}: {t} = {v}" for ln, t, v in whole))

    calls = [n for n in ast.walk(fn)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == '_ensure_data_handler']
    R.check("it calls _ensure_data_handler for all three services",
            len(calls) == 3, f"found {len(calls)}")

    services = set()
    for call in calls:
        if call.args and isinstance(call.args[0], ast.Attribute):
            services.add(call.args[0].attr)
    R.check("one call per service, no service left out",
            services == {'precipitation_service', 'vil_service',
                         'echo_top_service'},
            str(sorted(services)))


# --------------------------------------------------------------------------
# 3. The clobber was genuinely destructive -- SystemDatabase does not satisfy
#    the interface the store paths use. This is the part that makes the
#    finding a data-loss bug rather than a tidiness complaint, so it is
#    measured against the real classes, not asserted in prose.
# --------------------------------------------------------------------------

def test_a_systemdatabase_cannot_stand_in_for_a_data_handler():
    R.section("SystemDatabase does not satisfy the data-handler interface")

    from FMOFP.storage.DBM import SystemDatabase
    from FMOFP.local_messaging.routing.handlers.precipitation_data_handler import PrecipitationDataHandler
    from FMOFP.local_messaging.routing.handlers.vil_data_handler import VILDataHandler
    from FMOFP.local_messaging.routing.handlers.echo_top_data_handler import EchoTopDataHandler

    # The precise reason the live error was AttributeError at the gate.
    for missing in ('_verify_table_exists', '_init_database_table',
                    '_initialized', 'radar_db', 'store_echo_top_data'):
        R.check(f"SystemDatabase has no {missing}",
                not hasattr(SystemDatabase, missing))

    # The overlap that made the substitution survive review: these two do
    # exist on it, so a reader checking only the store call sees nothing
    # wrong.
    R.check("but it does have store_precipitation_data and store_vil_data, "
            "which is why the swap looked harmless",
            hasattr(SystemDatabase, 'store_precipitation_data')
            and hasattr(SystemDatabase, 'store_vil_data'))

    # The real handlers do have what their own store paths call.
    R.check("PrecipitationDataHandler has the gate method and the store",
            hasattr(PrecipitationDataHandler, '_verify_table_exists')
            and hasattr(PrecipitationDataHandler, '_init_database_table')
            and hasattr(PrecipitationDataHandler, 'store_precipitation_data'))
    R.check("VILDataHandler has the gate method and the store",
            hasattr(VILDataHandler, '_verify_table_exists')
            and hasattr(VILDataHandler, 'store_vil_data'))
    R.check("EchoTopDataHandler has the store SystemDatabase lacks",
            hasattr(EchoTopDataHandler, 'store_echo_top_data'))

    # EchoTopDataHandler deliberately does NOT have _verify_table_exists --
    # it uses _ensure_table. Recorded because it rules out hasattr() as the
    # predicate for "is this the right handler" and is why the fix tests the
    # class, not the method set.
    R.check("EchoTopDataHandler uses _ensure_table, not _verify_table_exists, "
            "so a method probe could not tell the three apart",
            hasattr(EchoTopDataHandler, '_ensure_table')
            and not hasattr(EchoTopDataHandler, '_verify_table_exists'))


def test_the_store_path_still_gates_on_the_method_that_was_missing():
    R.section("the store path still calls the method a SystemDatabase lacks")

    fn = _func(_tree(_PRECIP_SERVICE), '_store_precipitation_data')
    R.check("_store_precipitation_data is still there", fn is not None)
    if fn is None:
        return

    gate = [n for n in ast.walk(fn)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == '_verify_table_exists'
            and isinstance(n.func.value, ast.Attribute)
            and n.func.value.attr == 'data_handler']
    R.check("it calls self.data_handler._verify_table_exists() before storing",
            len(gate) >= 1,
            "if this fails the suite has lost its reason to exist; check "
            "whether the gate moved rather than deleting the test")

    stores = [n for n in ast.walk(fn)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == 'store_precipitation_data']
    R.check("and the store itself goes through data_handler too",
            len(stores) >= 1)


# --------------------------------------------------------------------------
# 4. _ensure_data_handler's four branches, exercised against fakes so the
#    behaviour is pinned without booting a database.
# --------------------------------------------------------------------------

class _Svc:
    pass


class _Right:
    def __init__(self, radar_db=None):
        self.radar_db = radar_db


class _Wrong:
    """Stands in for the SystemDatabase that used to land in the slot."""


def test_ensure_data_handler_leaves_a_correct_handler_alone():
    R.section("_ensure_data_handler: a correct handler is not rebuilt")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
    ensure = RadarMessageHandler._ensure_data_handler

    svc = _Svc()
    original = _Right('db-A')
    svc.data_handler = original
    ensure(svc, 'test', _Right, 'db-B')
    # Identity, not type: the two singleton handlers are shared, so replacing
    # one with an equal-but-new instance would still be a regression.
    R.check("the same object is still in the slot", svc.data_handler is original)
    R.check("and it was not re-pointed at the newer db",
            svc.data_handler.radar_db == 'db-A')


def test_ensure_data_handler_replaces_a_wrong_typed_occupant():
    R.section("_ensure_data_handler: the SystemDatabase case")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
    ensure = RadarMessageHandler._ensure_data_handler

    svc = _Svc()
    svc.data_handler = _Wrong()
    ensure(svc, 'test', _Right, 'db')
    R.check("a wrong-typed occupant is replaced",
            isinstance(svc.data_handler, _Right))
    R.check("and the replacement is built from the db it was given",
            svc.data_handler.radar_db == 'db')


def test_ensure_data_handler_fills_an_empty_slot():
    R.section("_ensure_data_handler: an empty slot is filled")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
    ensure = RadarMessageHandler._ensure_data_handler

    svc = _Svc()
    svc.data_handler = None
    ensure(svc, 'test', _Right, 'db')
    R.check("None is replaced", isinstance(svc.data_handler, _Right))

    # The attribute may be absent entirely: the services skip their __init__
    # body when radar_db is None at first construction, and never assign it.
    bare = _Svc()
    ensure(bare, 'test', _Right, 'db')
    R.check("a missing attribute is created",
            isinstance(getattr(bare, 'data_handler', None), _Right))


def test_ensure_data_handler_clears_rather_than_keeping_a_wrong_occupant():
    R.section("_ensure_data_handler: no radar_db to build from")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
    ensure = RadarMessageHandler._ensure_data_handler

    svc = _Svc()
    svc.data_handler = _Wrong()
    ensure(svc, 'test', _Right, None)
    R.check("the wrong occupant is cleared, not left in place",
            svc.data_handler is None)

    # A correct handler must survive even then -- losing it because radar_db
    # happened to be None on a re-bind would be the same bug in reverse.
    svc2 = _Svc()
    good = _Right('db')
    svc2.data_handler = good
    ensure(svc2, 'test', _Right, None)
    R.check("a correct handler survives a None radar_db",
            svc2.data_handler is good)


def test_ensure_data_handler_tolerates_a_missing_service():
    R.section("_ensure_data_handler: no service yet")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
    ensure = RadarMessageHandler._ensure_data_handler

    try:
        ensure(None, 'test', _Right, 'db')
        R.check("a None service is a no-op, not a crash", True)
    except Exception as exc:
        R.check("a None service is a no-op, not a crash", False, repr(exc))


# --------------------------------------------------------------------------
# 5. The probe trap, recorded as an assertion so the next person reading the
#    services off a handler instance knows why it looked empty.
# --------------------------------------------------------------------------

def test_a_fresh_radar_handler_tells_you_nothing_about_the_live_binding():
    R.section("where to read the binding from")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
        RadarMessageHandler, get_radar_message_handler)

    R.check("RadarMessageHandler defines no __new__ singleton, so the "
            "accessor hands out a new object each call",
            '__new__' not in RadarMessageHandler.__dict__)

    h = get_radar_message_handler()
    R.check("a freshly accessed handler has no precipitation_service, "
            "which is why inspecting one shows an empty slot",
            getattr(h, 'precipitation_service', 'absent') in (None, 'absent'))

    # The response services, by contrast, really are singletons -- which is
    # what makes a single clobber in set_async_handler reach every consumer.
    from FMOFP.local_messaging.routing.response_services.data_response_services.precipitation_response_service import PrecipitationResponseService
    from FMOFP.local_messaging.routing.response_services.data_response_services.vil_response_service import VILResponseService
    for cls in (PrecipitationResponseService, VILResponseService):
        R.check(f"{cls.__name__} is a singleton, so the slot it holds is "
                f"the one the whole system sees",
                '__new__' in cls.__dict__)


def main():
    print("=" * 60)
    print("  Radar response-service data_handler binding")
    print("=" * 60)

    for test in (test_the_detector_catches_the_shape_it_is_looking_for,
                 test_set_async_handler_no_longer_assigns_the_db_into_the_slot,
                 test_a_systemdatabase_cannot_stand_in_for_a_data_handler,
                 test_the_store_path_still_gates_on_the_method_that_was_missing,
                 test_ensure_data_handler_leaves_a_correct_handler_alone,
                 test_ensure_data_handler_replaces_a_wrong_typed_occupant,
                 test_ensure_data_handler_fills_an_empty_slot,
                 test_ensure_data_handler_clears_rather_than_keeping_a_wrong_occupant,
                 test_ensure_data_handler_tolerates_a_missing_service,
                 test_a_fresh_radar_handler_tells_you_nothing_about_the_live_binding):
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
