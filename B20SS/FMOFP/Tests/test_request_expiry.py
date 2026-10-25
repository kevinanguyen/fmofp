"""
Test suite: H8 and H7 — nothing ever closed the loop on an unanswered request.

H8. `RadarMessageHandler._start_cleanup_timer()` existed and had NO CALLER
anywhere, so pending requests were never expired and never retried. Because the
expiry path had never executed, its own bugs were undiscovered. Measured by
driving it directly: a non-mode_change request was appended to expired_uuids,
resent, and appended a second time -- deleted in the same pass that retried it.
`retries` was incremented on an object already being discarded, and each resend
built a fresh PendingRequest with retries=0, so max_retries=3 could never bind.
"Retry three times" was really "retry forever, once per expiry". Only
mode_change behaved as designed.

H7. `FCSMessageHandler` and `FMSMessageHandler` deliver timeout notifications
from a `threading.Timer` thread, and passed `asyncio.get_event_loop()` as the
target loop. get_event_loop() is per-thread: on the timer's thread it raises
`RuntimeError: There is no current event loop in thread 'Thread-N'` -- even
while a loop runs happily in another thread, which is this program's shape.
Measured before the fix, on the real handler: 0 of 3 notifications delivered,
`coroutine 'send_response' was never awaited`, and 2 of 3 requests left pending
because the exception aborted the batch after the first was already popped.

Run:  python3 -m FMOFP.Tests.test_request_expiry
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from FMOFP.Tests import code_only as _code_only  # shared helper

import asyncio
import inspect
import logging
import threading
import time
import traceback
import uuid as uuid_lib
import warnings


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


# ─────────────────────────── H8: the radar cleanup task ──────────────────────

def _radar(started=True):
    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
        RadarMessageHandler)

    class _AH:
        def __init__(self, s):
            self.started = s

        def is_healthy(self):
            return True

    h = RadarMessageHandler()
    h.async_handler = _AH(started)
    h.system_handler = object()
    h.started = True
    h.pending_requests.clear()
    return h


def _faithful_sender(h, log):
    """A send_request stub that behaves like the real one where it matters.

    The real one mints a fresh request_id and REGISTERS a pending entry under
    it. A stub that skips the registration measures itself rather than the
    retry budget -- which it did on the first attempt here, and reported the
    fix as not working.
    """
    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
        PendingRequest)

    async def send(radar_name, request_type, data=None, sync_response=False):
        rid = str(uuid_lib.uuid4())
        log.append(rid)
        h.pending_requests[rid] = PendingRequest(
            request_type, radar_name, time.time())
        return rid

    return send


def _expire_all(h):
    for req in h.pending_requests.values():
        req.timestamp = time.time() - 600.0


def test_cleanup_task_has_a_caller():
    R.section("H8 — the cleanup task is actually started")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
        RadarMessageHandler)

    src = inspect.getsource(RadarMessageHandler.start)
    R.check("start() calls _start_cleanup_timer",
            "_start_cleanup_timer" in src)
    R.check("...after setting self.started, which the task loops on",
            src.index("self.started = True") < src.index("_start_cleanup_timer"))
    R.check("stop() cancels it",
            "cleanup_timer.cancel()" in inspect.getsource(RadarMessageHandler.stop))

    h = _radar()
    R.check("a fresh handler has the attribute at all",
            hasattr(h, "cleanup_timer"), "it did not, before H8")

    async def live():
        h2 = _radar()
        h2.start()
        task = h2.cleanup_timer
        ok_started = task is not None and not task.done()
        # Starting twice must not leave two tasks running.
        h2._start_cleanup_timer()
        same = h2.cleanup_timer is task
        await asyncio.sleep(0.1)
        h2.stop()
        await asyncio.sleep(0.1)
        return ok_started, same, task.cancelled() or task.done()

    started, same, stopped = asyncio.run(live())
    R.check("start() leaves a live task", started)
    R.check("starting twice does not spawn a second one", same)
    R.check("stop() ends it", stopped)

    R.section("NON-TAUTOLOGICAL — without a running loop it declines, not raises")
    h3 = _radar()
    R.check("_start_cleanup_timer returns False off-loop",
            h3._start_cleanup_timer() is False)
    R.check("and leaves no task behind", h3.cleanup_timer is None)


def test_retry_budget_binds():
    R.section("H8 — the retry budget is carried forward and then given up on")

    async def run():
        h = _radar()
        resends = []
        h.send_request = _faithful_sender(h, resends)

        from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
            PendingRequest)
        h.pending_requests['first'] = PendingRequest(
            'status', 'weather_radar', time.time() - 600.0)
        budgets = []
        for _ in range(8):
            await h._cleanup_pending_requests()
            live = list(h.pending_requests.values())
            if not live:
                break
            budgets.append(live[0].retries)
            _expire_all(h)
        return budgets, len(resends), len(h.pending_requests)

    budgets, resends, left = asyncio.run(run())

    R.check("the budget climbs across resends rather than resetting",
            budgets == [1, 2, 3], f"budgets={budgets}")
    R.check("and it stops after max_retries resends", resends == 3,
            f"resends={resends}")
    R.check("leaving nothing pending", left == 0, f"pending={left}")


def test_every_request_type_follows_one_rule():
    R.section("H8 — mode_change is no longer the only type with a budget")

    async def run():
        from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
            PendingRequest)
        out = {}
        for kind in ('status', 'mode_change', 'data'):
            h = _radar()
            resends = []
            h.send_request = _faithful_sender(h, resends)
            h.pending_requests['x'] = PendingRequest(
                kind, 'weather_radar', time.time() - 600.0)
            for _ in range(8):
                await h._cleanup_pending_requests()
                if not h.pending_requests:
                    break
                _expire_all(h)
            out[kind] = len(resends)
        return out

    counts = asyncio.run(run())
    R.check("status, mode_change and data all stop at the same budget",
            len(set(counts.values())) == 1 and set(counts.values()) == {3},
            f"resends per type={counts}")


def test_expired_request_is_not_retried_and_dropped_at_once():
    R.section("NON-TAUTOLOGICAL — the pre-fix expression retried forever")

    async def run():
        """The old body, reproduced: append to expired_uuids unconditionally
        for a non-mode_change type, then resend, then append again. The resent
        request is a fresh PendingRequest, so the budget restarts at zero."""
        from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
            PendingRequest)
        h = _radar()
        resends = []
        send = _faithful_sender(h, resends)
        h.pending_requests['x'] = PendingRequest(
            'status', 'weather_radar', time.time() - 600.0)

        seen = []
        for _ in range(6):
            expired = []
            for uuid, request in list(h.pending_requests.items()):
                if not request.is_expired(time.time()):
                    continue
                expired.append(uuid)                      # unconditional
                if request.should_retry():
                    request.increment_retry()             # on a doomed object
                    await send(request.radar_name, request.request_type)
                else:
                    expired.append(uuid)
            for uuid in expired:
                h.pending_requests.pop(uuid, None)
            live = list(h.pending_requests.values())
            if not live:
                break
            seen.append(live[0].retries)
            _expire_all(h)
        return seen, len(resends)

    budgets, resends = asyncio.run(run())
    R.check("the budget never leaves zero, because each resend is a new object",
            budgets and set(budgets) == {0}, f"budgets={budgets}")
    R.check("so it resends on every pass without ever giving up",
            resends >= 6, f"resends={resends}")


def test_health_check_precondition_is_reachable():
    R.section("H8 — a stranded request can now reach the state is_healthy() tests")

    async def run():
        from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import (
            PendingRequest)
        h = _radar()
        resends = []
        h.send_request = _faithful_sender(h, resends)
        h.pending_requests['x'] = PendingRequest(
            'status', 'weather_radar', time.time() - 600.0)
        spent = []
        for _ in range(8):
            await h._cleanup_pending_requests()
            live = list(h.pending_requests.values())
            if not live:
                break
            spent.append(not live[0].should_retry())
            _expire_all(h)
        return any(spent)

    R.check("a request does reach 'expired and out of retries' at least once",
            asyncio.run(run()),
            "nothing incremented retries before H8, so this state was "
            "unreachable and the stuck-request count was always zero")


# ───────────────────── H7: timeout notifications are delivered ───────────────

def _drive_handler(module_name, getter, make_request, response_attr):
    """Start a handler the way the system does, strand requests, let the real
    timer fire, and report what was delivered."""
    import importlib
    mod = importlib.import_module(module_name)
    handler = getattr(mod, getter)()

    delivered = []

    class _Spy:
        async def send_response(self, request_id, response):
            status = response.get('status') if isinstance(response, dict) else None
            delivered.append((request_id, status))
            return True

    setattr(handler, response_attr, _Spy())

    async def main():
        handler.started = False
        handler.start()                 # awaited context -> a loop is running
        captured = getattr(handler, '_loop', None) is not None
        handler.pending_requests.clear()
        for i in range(3):
            rid = f"req-{i}"
            req = make_request(mod, rid)
            req.retry_count = req.max_retries      # budget spent -> notify
            handler.pending_requests[rid] = req
        planted = len(handler.pending_requests)
        await asyncio.sleep(3.0)
        left = len(handler.pending_requests)
        handler.stop()
        return captured, planted, list(delivered), left

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return asyncio.run(main())


def _expired_request(mod, rid):
    return mod.PendingRequest(rid, "fms_modeChange", time.time() - 600.0)


def test_fms_timeout_notifications_are_delivered():
    R.section("H7 — FMS notifies every expired request, not none of them")

    captured, planted, delivered, left = _drive_handler(
        'FMOFP.local_messaging.routing.handlers.system_message_handlers.FMSMessageHandler',
        'get_fms_message_handler', _expired_request, 'response_service')

    R.check("start() captured the loop to notify on", captured)
    R.check("all three expired requests were notified", len(delivered) == 3,
            f"delivered={delivered}")
    R.check("each was told it timed out",
            all(s == "ERROR" for _r, s in delivered), f"delivered={delivered}")
    R.check("and none were left pending", left == 0,
            f"{left} of {planted} still pending")


def test_notifier_never_raises_into_the_batch():
    R.section("H7 — a failed notification cannot abort the rest of the batch")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.FMSMessageHandler import (
        get_fms_message_handler)
    h = get_fms_message_handler()

    h._loop = None
    h.response_service = object()
    R.check("no loop -> False, not an exception",
            h._notify_request_timed_out("r", {"status": "ERROR"}) is False)

    class _Boom:
        async def send_response(self, request_id, response):
            raise RuntimeError("downstream exploded")

    async def with_loop():
        h._loop = asyncio.get_running_loop()
        h.response_service = _Boom()
        # The coroutine is submitted; the failure happens on the loop, so the
        # submission itself still succeeds. What matters is that nothing raises
        # back into the caller.
        return h._notify_request_timed_out("r", {"status": "ERROR"})

    try:
        asyncio.run(with_loop())
        R.check("a failing response service does not raise into the caller", True)
    except Exception as e:
        R.check("a failing response service does not raise into the caller",
                False, f"{type(e).__name__}: {e}")

    h._loop = None
    h.response_service = None
    R.check("no response service -> False, not an exception",
            h._notify_request_timed_out("r", {"status": "ERROR"}) is False)


def test_pre_fix_get_event_loop_raises_on_a_timer_thread():
    R.section("NON-TAUTOLOGICAL — get_event_loop() on a timer thread raises")

    result = {}

    def body():
        try:
            asyncio.get_event_loop()
            result['outcome'] = "no error"
        except RuntimeError as e:
            result['outcome'] = f"RuntimeError: {e}"

    # A loop running in ANOTHER thread, which is the real situation: it does
    # not help, because get_event_loop() is per-thread.
    def runner():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_forever()

    threading.Thread(target=runner, daemon=True).start()
    time.sleep(0.3)
    t = threading.Timer(0.01, body)
    t.start()
    t.join(5.0)

    R.check("it raises even with a loop live in another thread",
            str(result.get('outcome', '')).startswith("RuntimeError"),
            f"outcome={result.get('outcome')!r}")

    R.section("H7 — and neither handler calls it any more")
    import FMOFP.local_messaging.routing.handlers.system_message_handlers.FCSMessageHandler as fcs
    import FMOFP.local_messaging.routing.handlers.system_message_handlers.FMSMessageHandler as fms

    R.check("the comment stripper flags a real call",
            "get_event_loop" in _code_only(
                "def f():\n    loop = asyncio.get_event_loop()\n"))
    R.check("...and not one merely named in a comment",
            "get_event_loop" not in _code_only(
                "def f():\n    # used to call asyncio.get_event_loop()\n    pass\n"))

    for mod, tag in ((fcs, "FCS"), (fms, "FMS")):
        cls = getattr(mod, f"{tag}MessageHandler")
        cleanup = _code_only(inspect.getsource(cls._cleanup_pending_requests))
        R.check(f"{tag}'s cleanup no longer calls get_event_loop()",
                "get_event_loop" not in cleanup,
                "still called in code, not just named in a comment")
        R.check(f"{tag} notifies through the captured loop",
                "_notify_request_timed_out" in cleanup)
        R.check(f"{tag} captures it in start()",
                "_remember_loop" in inspect.getsource(cls.start))



# ───────────── The cleanup timer, and what its log claimed ──────────────────

def _fms():
    import importlib
    mod = importlib.import_module(
        'FMOFP.local_messaging.routing.handlers.system_message_handlers'
        '.FMSMessageHandler')
    return mod


def test_pre_fix_rearm_guard_could_never_pass():
    R.section("NON-TAUTOLOGICAL — is_alive() is True inside the Timer's own target")

    seen = {}

    def target():
        seen['inside'] = timer.is_alive()

    timer = threading.Timer(0.01, target)
    timer.start()
    timer.join(5.0)

    R.check("a Timer reports itself alive from inside its own target",
            seen.get('inside') is True, f"seen={seen}")
    R.check("so `if self.cleanup_timer.is_alive(): return` always returned "
            "early when re-armed from the timer thread", seen.get('inside') is True)


def test_cleanup_timer_rearms():
    R.section("The cleanup timer runs repeatedly, not once")

    mod = _fms()
    import inspect
    src = inspect.getsource(mod.FMSMessageHandler._start_cleanup_timer)
    R.check("the re-arm guard excludes the thread doing the re-arming",
            "threading.current_thread()" in src,
            "without this it refuses to re-arm from inside the timer")
    R.check("and it declines to re-arm after stop()",
            "if not self.started" in src)

    class _Bus:
        async def send_message(self, cw, dw, rid, md):
            return {'ok': True}

    passes = []

    async def run():
        h = mod.get_fms_message_handler()
        h.sendMsg = _Bus()
        h.response_service = None
        h.started = False
        h.start()
        real = h._cleanup_pending_requests

        def counting():
            passes.append(time.monotonic())
            return real()

        h._cleanup_pending_requests = counting
        # Re-arm so the counting wrapper is what the next timer calls.
        if h.cleanup_timer:
            h.cleanup_timer.cancel()
        h.cleanup_timer = None
        h._start_cleanup_timer()
        await asyncio.sleep(4.0)

        # Poll, rather than sampling the slot at one instant.
        #
        # This was a single read of h.cleanup_timer.is_alive() taken after
        # exactly 4.0s -- a whole multiple of the 1s interval, so it landed
        # on a firing boundary, which is the one place the slot can briefly
        # hold a timer that is not yet alive. CI failed on it: "FAIL and a
        # timer is still pending at the end", alone, with the other two
        # checks in this same test passing. _start_cleanup_timer now starts
        # the timer before publishing it, which took the window from 0.16%
        # of wall time (43,196 of 27,462,452 samples) down to roughly one
        # observation per firing -- narrower, but not provably zero.
        #
        # The claim worth making is "the chain is still pending", not "the
        # slot was alive at this exact microsecond". A chain that had
        # stopped re-arming would never show an alive timer again, so
        # polling asserts the same thing without racing the hand-off. The
        # budget is well over one interval, so a live chain cannot miss it.
        alive = False
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            timer = h.cleanup_timer
            if timer is not None and timer.is_alive():
                alive = True
                break
            await asyncio.sleep(0.01)

        h.stop()
        await asyncio.sleep(0.2)
        after_stop = len(passes)
        await asyncio.sleep(1.5)
        return alive, after_stop

    alive, after_stop = asyncio.run(run())

    R.check("cleanup ran more than once in 4s", len(passes) >= 3,
            f"passes={len(passes)} -- it ran exactly once before this fix")
    R.check("and a timer is still pending at the end", alive)
    # Non-tautological: the poll above must be capable of answering False.
    # A handler that was never started has no chain, and the same poll
    # against it has to come back empty within the same budget.
    def _poll_stopped():
        idle = mod.FMSMessageHandler.__new__(mod.FMSMessageHandler)
        idle.cleanup_timer = None
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            timer = idle.cleanup_timer
            if timer is not None and timer.is_alive():
                return True
            time.sleep(0.01)
        return False

    R.check("...and the same poll reports False for a handler with no chain",
            _poll_stopped() is False)
    R.check("stop() ends the chain rather than leaving it re-arming",
            len(passes) <= after_stop + 1,
            f"{len(passes)} passes total, {after_stop} by the time stop returned")


def test_the_log_no_longer_claims_a_retry():
    R.section("The timeout log says what happened, and what did not")

    mod = _fms()
    import inspect
    cleanup = _code_only(
        inspect.getsource(mod.FMSMessageHandler._cleanup_pending_requests))

    R.check("nothing claims to be 'Retrying request'",
            "Retrying request" not in cleanup,
            "it logged a retry it never performed")
    # Raw source, NOT _code_only(): retry_count is interpolated inside an
    # f-string, and _code_only strips STRING tokens, so it had already been
    # thrown away. _code_only is for "is this still CALLED"; it can say nothing
    # about logged text. (Third time in this session a check of mine measured
    # the harness instead of the code.)
    raw_cleanup = inspect.getsource(
        mod.FMSMessageHandler._cleanup_pending_requests)
    R.check("the expiry line interpolates the real count",
            "{request.retry_count}" in raw_cleanup,
            "it printed max_retries, so a first-window expiry said '3 retries'")

    captured = []

    class _Cap(logging.Handler):
        def emit(self, record):
            try:
                captured.append(record.getMessage())
            except Exception:
                pass

    class _Bus:
        async def send_message(self, cw, dw, rid, md):
            return {'ok': True}

    sends = []

    async def run():
        h = mod.get_fms_message_handler()
        h.sendMsg = _Bus()

        notified = []

        class _Spy:
            async def send_response(self, rid, resp):
                notified.append((rid, resp.get('status')))
                return True

        h.response_service = _Spy()
        h.started = False
        h.start()
        real_send = h.send_request

        async def tracking(*a, **k):
            sends.append((a, k))
            return await real_send(*a, **k)

        h.send_request = tracking
        h.pending_requests.clear()
        # Already out of windows, so this expires on the next pass.
        req = mod.PendingRequest('probe', 'mode_change', time.time() - 600.0,
                                 system_name='fms', data={'mode': 'CRUISE'})
        req.retry_count = req.max_retries
        h.pending_requests['probe'] = req

        cap = _Cap()
        logging.getLogger().addHandler(cap)
        try:
            await asyncio.sleep(3.0)
        finally:
            logging.getLogger().removeHandler(cap)
        h.stop()
        return notified

    notified = asyncio.run(run())

    ours = [m for m in captured if 'probe' in m]
    R.check("the expired request was notified", len(notified) == 1,
            f"notified={notified}")
    R.check("and no send_request was issued on its behalf", not sends,
            f"sends={len(sends)}")
    R.check("the log reports the real window count, not a bare max_retries",
            any("3 of 3 windows" in m for m in ours),
            f"lines={[m[:70] for m in ours]}")


def test_pending_request_can_describe_itself():
    R.section("A pending request records enough to be re-issued")

    mod = _fms()
    req = mod.PendingRequest('r', 'mode_change', time.time(),
                             system_name='flightManagementSystem',
                             data={'mode': 'CRUISE'})
    R.check("it stores the system it was sent to", req.system_name ==
            'flightManagementSystem')
    R.check("and the payload", req.data == {'mode': 'CRUISE'})

    before = req.timestamp
    time.sleep(0.02)
    req.increment_retry()
    R.check("increment_retry restamps, so a retried request gets a new window",
            req.timestamp > before,
            "without this the 1s tick burned all three windows in 3s")

    import inspect
    sig = inspect.signature(mod.FMSMessageHandler.send_request)
    R.check("send_request can inherit a retry budget",
            "inherited_retries" in sig.parameters)

    # STEALTH, not CRUISE. This fixture used to send {"mode": "CRUISE"} and
    # get a request ID back, which is exactly the defect story C25.1's FMS
    # half turned up: CRUISE is not one of the FMS's five modes
    # (FlightManagementSystem.set_mode's valid_modes are NORMAL, COMBAT,
    # STEALTH, TRAINING, EMERGENCY), and the mode packer's
    # `.get(name.upper(), 0)` default turned any unrecognised name into 0,
    # which is NORMAL. So this fixture was transmitting a command to enter
    # NORMAL and reading it as a successful send of CRUISE. The packer
    # refuses an unknown mode now, so the payload has to be a real one --
    # which is what this test wanted all along; it is about the retry
    # budget, not about mode validity.
    async def seeded():
        class _Bus:
            async def send_message(self, cw, dw, rid, md):
                return {'ok': True}
        h = mod.get_fms_message_handler()
        h.sendMsg = _Bus()
        h.started = True
        h.pending_requests.clear()
        rid = await h.send_request("flightManagementSystem", "mode_change",
                                   {"mode": "STEALTH"}, inherited_retries=2)
        entry = h.pending_requests.get(rid)
        return entry.retry_count if entry else None, entry

    count, entry = asyncio.run(seeded())
    R.check("and the new entry carries it", count == 2, f"retry_count={count}")
    R.check("while still recording its own inputs",
            entry is not None and entry.system_name == "flightManagementSystem")

    # And the mode this fixture used to send is now refused outright, with no
    # request ID and no pending entry left behind -- rather than being
    # transmitted as a command to enter NORMAL.
    async def bogus():
        class _Bus:
            async def send_message(self, cw, dw, rid, md):
                return {'ok': True}
        h = mod.get_fms_message_handler()
        h.sendMsg = _Bus()
        h.started = True
        h.pending_requests.clear()
        rid = await h.send_request("flightManagementSystem", "mode_change",
                                   {"mode": "CRUISE"})
        return rid, len(h.pending_requests)

    rid, left_pending = asyncio.run(bogus())
    R.check("a mode the FMS does not have returns no request ID",
            rid is None, repr(rid))
    R.check("...and leaves nothing pending behind it",
            left_pending == 0, str(left_pending))


def test_every_pending_mutation_is_guarded():
    R.section("The lock covers every side, not just the cleanup pass")

    import ast
    import inspect

    def unguarded(path):
        src = open(path, encoding='utf-8').read()
        lines = src.split('\n')
        out = []
        for i, line in enumerate(lines):
            if 'self.pending_requests' not in line:
                continue
            stripped = line.strip()
            mutates = (stripped.startswith('self.pending_requests[')
                       or '.pop(' in stripped or stripped.startswith('del self.pending_requests')
                       or '.items()' in stripped or '.clear()' in stripped)
            if not mutates or stripped.startswith('#'):
                continue
            indent = len(line) - len(line.lstrip())
            guarded = False
            for j in range(i - 1, -1, -1):
                prev = lines[j]
                if not prev.strip():
                    continue
                pind = len(prev) - len(prev.lstrip())
                if pind < indent:
                    if 'with self.lock' in prev:
                        guarded = True
                        break
                    if prev.strip().startswith(('def ', 'async def ')):
                        break
                    indent = pind
            if not guarded:
                out.append((i + 1, stripped[:58]))
        return out

    base = ('FMOFP/local_messaging/routing/handlers/system_message_handlers/'
            '%sMessageHandler.py')
    for tag in ('FMS', 'FCS'):
        path = os.path.join(_B20SS, base % tag)
        left = [(n, t) for n, t in unguarded(path)
                if 'self.pending_requests = {}' not in t]
        R.check(f"{tag}: no unguarded pending_requests mutation", not left,
                f"{left}")

    R.section("NON-TAUTOLOGICAL — the scanner finds a planted unguarded write")
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False,
                                     encoding='utf-8') as fh:
        fh.write("class C:\n"
                 "    def f(self, rid, r):\n"
                 "        self.pending_requests[rid] = r\n"
                 "    def g(self, rid):\n"
                 "        with self.lock:\n"
                 "            self.pending_requests.pop(rid, None)\n")
        probe = fh.name
    try:
        found = unguarded(probe)
        R.check("it flags the unguarded one and not the guarded one",
                len(found) == 1 and found[0][0] == 3, f"found={found}")
    finally:
        os.unlink(probe)


def main():
    print("=" * 60)
    print("  H8 / H7: request expiry and timeout notification")
    print("=" * 60)

    for test in (test_cleanup_task_has_a_caller,
                 test_retry_budget_binds,
                 test_every_request_type_follows_one_rule,
                 test_expired_request_is_not_retried_and_dropped_at_once,
                 test_health_check_precondition_is_reachable,
                 test_pre_fix_get_event_loop_raises_on_a_timer_thread,
                 test_notifier_never_raises_into_the_batch,
                 test_fms_timeout_notifications_are_delivered,
                 test_pre_fix_rearm_guard_could_never_pass,
                 test_pending_request_can_describe_itself,
                 test_every_pending_mutation_is_guarded,
                 test_the_log_no_longer_claims_a_retry,
                 test_cleanup_timer_rearms):
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
