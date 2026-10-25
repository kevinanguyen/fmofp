"""
Test suite: the bus send path could not finish.

Three findings, one story.

1. THE BLOCK-TRANSFER SPLIT NEVER TERMINATED. _send_message routes any message
   with more than 33 frames to _send_large_message, which chunks it. A chunk was
   built as [command_word] + [sequence_word, total_word] + 32 payload frames =
   35 frames -- above the same 33-frame threshold. So every chunk was itself
   "large" and re-entered the split, producing another 35-frame chunk. Computed
   over the real arithmetic: 35 splits into 35 and 5, forever. A fixed point,
   not a reduction. ANY bus message over 33 frames never completed. Reproduced
   against a peer that accepts and never reads: eleven nested
   _send_large_message frames on the stack and eleven sockets held open.

   The two sequence words are data words, so they count against the standard's
   32. Payload per chunk is 30, and a chunk is exactly 33 frames.

2. NOTHING WAS BOUNDED. The bus adapter's socket had no timeout anywhere, and
   BC_send_message / RT_send_message called future.result() with none either.
   Measured after the split fix: the caller returned False at 30.2s instead of
   never -- but the WORKER ran on, and the process would not exit, because
   ThreadPoolExecutor joins its workers at interpreter shutdown. A transfer is
   one connect-and-send cycle per chunk, thousands of them, and a per-socket
   timeout bounds each separately, not the total. Hence a deadline for the
   whole transfer: 21.3s and a clean exit.

3. ONE CALL SITE TIMED OUT BY CONSTRUCTION. AsyncMessageHandler._handle_message
   created a NEW event loop, set it, and submitted to it with
   run_coroutine_threadsafe -- a loop nobody runs. Nothing could execute the
   coroutine, so future.result(timeout=5.0) burned the full five seconds and
   raised, every time. Measured: TimeoutError after 5.0s with "coroutine was
   never awaited", against 0.001s for the replacement.

Six other future.result() calls were checked and are NOT defects: they sit in
add_done_callback handlers, where the future is already complete.

Run:  python3 -m FMOFP.Tests.test_send_path_bounds
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from FMOFP.Tests import code_only  # shared source-inspection helper

import inspect
import subprocess
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

_PRELUDE = f"""
import os, socket, sys, threading, time
sys.path.insert(0, {_B20SS!r})
sys.path.insert(0, {os.path.join(_B20SS, 'FMOFP')!r})
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
"""


def _run_isolated(body, timeout=60.0):
    """Run `body` in a subprocess. Returns (finished, stdout).

    finished is False when it had to be killed -- which is what an unbounded
    send looks like from outside, and the only safe way to observe one.
    """
    script = _PRELUDE + textwrap.dedent(body)
    try:
        proc = subprocess.run([sys.executable, "-c", script],
                              capture_output=True, text=True,
                              timeout=timeout, cwd=_B20SS)
    except subprocess.TimeoutExpired as e:
        return False, (e.stdout if isinstance(e.stdout, str) else "") or ""
    return True, proc.stdout


# ───────────────────── 1. the split has to shrink the message ───────────────

def _largest_chunk(n_frames, payload_per_chunk, header_words=2):
    """The frame count of the biggest message _send_large_message emits."""
    data_frames = n_frames - 1                     # frames[1:] after the first word
    num = (data_frames + payload_per_chunk - 1) // payload_per_chunk
    return max(1 + header_words +
               (min((i + 1) * payload_per_chunk, data_frames) - i * payload_per_chunk)
               for i in range(num))


def test_block_transfer_split_terminates():
    R.section("The block-transfer split reduces the message")

    import FMOFP.MIL_STD_1553B.Bus_Controller.BC_connect.BC_socket as bc

    R.check("the standard's limit is named, not a literal",
            bc.MAX_DATA_WORDS_PER_MESSAGE == 32
            and bc.MAX_FRAMES_PER_MESSAGE == 33)
    R.check("a chunk's two sequence words count against it",
            bc.MAX_PAYLOAD_FRAMES_PER_CHUNK ==
            bc.MAX_DATA_WORDS_PER_MESSAGE - bc.BLOCK_TRANSFER_HEADER_WORDS == 30)

    offenders = []
    for n in (34, 35, 64, 100, 1000, 400000):
        biggest = _largest_chunk(n, bc.MAX_PAYLOAD_FRAMES_PER_CHUNK,
                                 bc.BLOCK_TRANSFER_HEADER_WORDS)
        if biggest > bc.MAX_FRAMES_PER_MESSAGE:
            offenders.append((n, biggest))
    R.check("no input size produces a chunk that would be re-split",
            not offenders, f"{offenders}")

    R.check("and a chunk fills the standard's message exactly",
            _largest_chunk(400000, bc.MAX_PAYLOAD_FRAMES_PER_CHUNK,
                           bc.BLOCK_TRANSFER_HEADER_WORDS)
            == bc.MAX_FRAMES_PER_MESSAGE)

    R.section("NON-TAUTOLOGICAL — the old constant was a fixed point")
    n = 400000
    seen = []
    for _ in range(5):
        n = _largest_chunk(n, 32)        # the pre-fix payload size
        seen.append(n)
    R.check("32 payload frames produces 35-frame chunks forever",
            set(seen) == {35} and 35 > bc.MAX_FRAMES_PER_MESSAGE,
            f"largest chunk per level: {seen}")


def test_both_senders_carry_the_fix():
    R.section("BC and RT are near-duplicates, so both are checked")

    import FMOFP.MIL_STD_1553B.Bus_Controller.BC_connect.BC_socket as bc
    import FMOFP.MIL_STD_1553B.Remote_Terminal.RT_connect.RT_socket as rt

    for mod, tag in ((bc, "BC"), (rt, "RT")):
        for name in ("MAX_DATA_WORDS_PER_MESSAGE", "MAX_FRAMES_PER_MESSAGE",
                     "MAX_PAYLOAD_FRAMES_PER_CHUNK", "BLOCK_TRANSFER_HEADER_WORDS",
                     "SEND_COMPLETION_TIMEOUT_S", "BLOCK_TRANSFER_TIMEOUT_S"):
            R.check(f"{tag} declares {name}", hasattr(mod, name))

        src = inspect.getsource(mod)
        R.check(f"{tag} refuses a chunk that would be re-split",
                "would be split again forever" in src)
        R.check(f"{tag} gives the whole transfer a deadline",
                "transfer_deadline" in src)
        R.check(f"{tag}'s split threshold is the named constant, not 33",
                "> MAX_FRAMES_PER_MESSAGE" in src)
        R.check(f"{tag} bounds the caller's wait",
                "future.result(timeout=SEND_COMPLETION_TIMEOUT_S)" in src)

    R.check("the two senders agree on the limits",
            (bc.MAX_FRAMES_PER_MESSAGE, bc.MAX_PAYLOAD_FRAMES_PER_CHUNK) ==
            (rt.MAX_FRAMES_PER_MESSAGE, rt.MAX_PAYLOAD_FRAMES_PER_CHUNK))


# ─────────────────────────── 2. the socket is bounded ───────────────────────

def test_bus_adapter_sets_a_socket_timeout():
    R.section("The transport socket has a timeout at all")

    import FMOFP.MIL_STD_1553B.bus_adapter as ba

    R.check("there is a declared socket timeout",
            isinstance(getattr(ba, "BUS_SOCKET_TIMEOUT_S", None), float)
            and 0 < ba.BUS_SOCKET_TIMEOUT_S <= 60)

    # code_only(): the comment above the settimeout call explains that
    # connect() has the same exposure, so an ordering check against raw source
    # finds "connect(" FIRST and fails on correct code. Fourth time that shape
    # of mistake has been made here, which is why the helper is shared.
    src = code_only(inspect.getsource(ba.SocketBusAdapter.transmit))
    R.check("the stripper flags a real call but not a commented one",
            "connect" in code_only("x.connect()")
            and "connect" not in code_only("# calls connect() one day\npass"))
    R.check("transmit applies it before connecting",
            "settimeout" in src and src.index("settimeout") < src.index("connect"))
    R.check("and a timeout is reported rather than swallowed as a generic error",
            "timeout" in src)

    finished, out = _run_isolated("""
        import FMOFP.MIL_STD_1553B.bus_adapter as ba
        applied = []
        real_socket = socket.socket
        class _Spy(real_socket):
            def settimeout(self, t):
                applied.append(t)
                return super().settimeout(t)
        socket.socket = _Spy
        a = ba.SocketBusAdapter('bc', peer_host='127.0.0.1', peer_port=1)
        a.transmit(b'x')          # refused; we only care that a timeout was set
        socket.socket = real_socket
        print(f"TIMEOUTS={applied}")
    """, timeout=40.0)
    R.check("a real transmit sets it on the real socket",
            finished and "TIMEOUTS=[" in out and "None" not in out.split("TIMEOUTS=")[-1],
            out.strip()[-80:])


def test_a_deaf_peer_no_longer_parks_the_caller():
    R.section("A peer that accepts and never reads cannot park the sender")

    finished, out = _run_isolated("""
        import FMOFP.MIL_STD_1553B.bus_adapter as ba
        import FMOFP.MIL_STD_1553B.Bus_Controller.BC_connect.BC_socket as bcmod

        # Short deadlines so this is a test, not a wait.
        bcmod.BLOCK_TRANSFER_TIMEOUT_S = 3.0
        bcmod.SEND_COMPLETION_TIMEOUT_S = 8.0

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2048)
        srv.bind(('127.0.0.1', 0)); srv.listen(5)
        port = srv.getsockname()[1]
        held = []
        def deaf():
            while True:
                try:
                    c, _ = srv.accept(); held.append(c)
                except OSError:
                    return
        threading.Thread(target=deaf, daemon=True).start()

        s = bcmod.BC_sender()
        s._bus_adapter = ba.SocketBusAdapter('bc', peer_host='127.0.0.1',
                                             peer_port=port)
        big = {'frames': ['1000000000000000000' + str(i % 10)
                          for i in range(200000)]}
        t0 = time.monotonic()
        result = s.BC_send_message(big)
        print(f"RETURNED={result} AFTER={time.monotonic()-t0:.1f}")
    """, timeout=45.0)

    R.check("the call returns at all", finished and "RETURNED=" in out,
            "it never returned" if not finished else out.strip()[-80:])
    if "RETURNED=" in out:
        line = [l for l in out.splitlines() if l.startswith("RETURNED=")][0]
        result, after = line.split()
        secs = float(after.split("=")[1])
        R.check("it reports failure rather than claiming success",
                result == "RETURNED=False", line)
        R.check("and it does so within its deadline, not eventually",
                secs < 12.0, f"{secs}s")
    R.check("the process exits once the call returns, so no worker is wedged",
            finished)


# ──────────────────── 3. submitting to a loop nobody runs ───────────────────

def test_handler_dispatch_does_not_time_out_by_construction():
    R.section("A coroutine handler is run, not submitted into the void")

    import FMOFP.local_messaging.routing.handlers.sync_handler.AsyncMessageHandler as am
    src = inspect.getsource(am.AsyncMessageHandler._handle_message)

    R.check("it no longer creates a loop it never runs",
            "asyncio.new_event_loop()" not in src,
            "a loop that is created, set and never run cannot execute anything")
    R.check("a loop is only submitted to while it is running",
            "is_running()" in src)
    R.check("and otherwise the coroutine is run here, bounded",
            "asyncio.run(" in src and "wait_for" in src)

    R.section("NON-TAUTOLOGICAL — the old shape always timed out")
    finished, out = _run_isolated("""
        import asyncio, warnings
        warnings.simplefilter("ignore")
        async def handler(d):
            return "ok"

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        t0 = time.monotonic()
        fut = asyncio.run_coroutine_threadsafe(handler({}), loop)
        try:
            fut.result(timeout=2.0)
            print("OLD=returned")
        except Exception as e:
            print(f"OLD={type(e).__name__} after {time.monotonic()-t0:.1f}s")
        loop.close()

        t0 = time.monotonic()
        got = asyncio.run(asyncio.wait_for(handler({}), timeout=2.0))
        print(f"NEW={got!r} after {time.monotonic()-t0:.3f}s")
    """, timeout=30.0)

    R.check("submitting to an unrun loop times out", "OLD=TimeoutError" in out,
            out.strip()[-80:])
    R.check("running it instead returns immediately",
            "NEW='ok'" in out and "after 0.0" in out.split("NEW=")[-1],
            out.strip()[-60:])


def test_done_callback_results_are_not_defects():
    R.section("The six router future.result() calls are correct, not blocking")

    router = os.path.join(_B20SS, 'FMOFP', 'local_messaging', 'routing',
                          'display_outgoing_router.py')
    lines = open(router, encoding='utf-8').read().split('\n')
    bare = []
    for i, line in enumerate(lines):
        if '.result()' not in line or line.strip().startswith('#'):
            continue
        # Walk back for the enclosing def; a done-callback takes the future as
        # its argument and is registered with add_done_callback.
        inside_callback = any(
            'def handle_result' in lines[j] for j in range(max(0, i - 12), i))
        if not inside_callback:
            bare.append((i + 1, line.strip()[:50]))
    R.check("every one of them is inside a done-callback", not bare, f"{bare}")
    R.check("...and the file registers them as such",
            "add_done_callback" in "\n".join(lines))


def main():
    print("=" * 60)
    print("  The bus send path: termination and bounds")
    print("=" * 60)

    for test in (test_block_transfer_split_terminates,
                 test_both_senders_carry_the_fix,
                 test_bus_adapter_sets_a_socket_timeout,
                 test_handler_dispatch_does_not_time_out_by_construction,
                 test_done_callback_results_are_not_defects,
                 test_a_deaf_peer_no_longer_parks_the_caller):
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
