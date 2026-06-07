# -*- coding: utf-8 -*-
"""
Progress reporting for FusionRobotExporter.

Wraps Fusion 360's native ProgressDialog and adds object counting, percentage,
elapsed time and a time estimate (ETA) based on the recent throughput over a
sliding window, smoothed with an EMA. Shared by every exporter so the user sees
a single, consistent progress window after pressing OK.

Typical use inside an exporter::

    from ...core.progress import ProgressReporter, count_link_occurrences

    progress = ProgressReporter(ui)
    progress.start(total, 'Exporting URDF (ROS2)')
    ...
    progress.step('part_name')          # call once per processed object
    if progress.is_cancelled():
        progress.finish()
        return False, 'Export cancelled by user.'
    ...
    progress.finish()

The reporter is a safe no-op when no ``ui`` is provided (e.g. unit tests), so the
exporters keep working when called outside of Fusion.
"""

import time
from collections import deque

try:
    import adsk.core
except Exception:  # pragma: no cover - allows importing outside Fusion
    adsk = None


class ProgressReporter:
    """Thin, fail-safe wrapper around adsk.core.ProgressDialog."""

    def __init__(self, ui=None, unit='objects'):
        self.ui = ui
        self.dialog = None
        self.total = 1
        self.done = 0
        self.start_time = None
        self.title = 'Exporting'
        self._phase = ''
        self.unit = unit
        # Throttle dialog repaints so huge body counts don't drown in UI updates.
        self._last_refresh = 0.0
        self._min_interval = 0.1  # seconds between repaints

        # --- ETA estimation -------------------------------------------------
        # Recent throughput is measured over a sliding time window. The window
        # is intentionally long enough to span a full heavy cycle (body merge +
        # STL export + inertia of a merged group), so the rate already includes
        # those costs instead of jumping every time a phase ends.
        self._eta_window = 15.0     # seconds of history for the recent rate
        self._eta_alpha = 0.25      # EMA smoothing of the displayed ETA (0..1)
        self._eta_warmup = 1.5      # don't show a number before this (seconds)
        self._samples = deque(maxlen=8192)  # (timestamp, done) within the window
        self._eta_ema = None        # smoothed ETA in seconds

    # -- lifecycle ---------------------------------------------------------
    def start(self, total, title='Exporting', message='Preparing...', unit=None):
        """Open the progress window for ``total`` expected steps."""
        self.total = max(int(total), 1)
        self.done = 0
        self.start_time = time.time()
        self.title = title
        self._phase = ''
        self._last_refresh = 0.0
        self._samples.clear()
        self._eta_ema = None
        if unit is not None:
            self.unit = unit
        if not self.ui:
            return
        try:
            self.dialog = self.ui.createProgressDialog()
            self.dialog.isCancelButtonShown = True
            self.dialog.cancelButtonText = 'Cancel'
            # show(title, message, minimumValue, maximumValue, delayInSeconds)
            self.dialog.show(title, message, 0, self.total, 0)
            self._do_events()
        except Exception:
            self.dialog = None

    def set_total(self, total):
        """Adjust the maximum after start() if a better count becomes known."""
        self.total = max(int(total), 1)
        if self.dialog:
            try:
                self.dialog.maximumValue = self.total
            except Exception:
                pass

    def set_phase(self, phase):
        """Set the leading label shown above the counter (e.g. 'Exporting meshes')."""
        self._phase = phase or ''
        # Force a repaint so the new phase (often before a long blocking call)
        # shows up immediately.
        self._refresh('', force=True)

    def step(self, label=''):
        """Advance the counter by one and refresh the window text (throttled)."""
        self.done += 1
        # Record a sample for the recent-rate ETA. A long gap since the previous
        # step (a heavy un-stepped phase such as STL export / inertia ran) is
        # kept as a single slow sample inside the window, so the estimated rate
        # stays realistic instead of pretending the gap never happened.
        now = time.time()
        self._samples.append((now, self.done))
        cutoff = now - self._eta_window
        while len(self._samples) > 2 and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self._refresh(label, force=(self.done >= self.total))

    def tick(self, label=''):
        """Keep the UI alive during un-budgeted sub-loops without consuming a
        step (e.g. while merging bodies that are counted elsewhere)."""
        self._refresh(label, force=False)

    def is_cancelled(self):
        if not self.dialog:
            return False
        try:
            return self.dialog.wasCancelled
        except Exception:
            return False

    def finish(self):
        if self.dialog:
            try:
                self.dialog.hide()
            except Exception:
                pass
        self.dialog = None

    # -- internals ---------------------------------------------------------
    def _refresh(self, label='', force=False):
        if not self.dialog:
            return
        now = time.time()
        if not force and (now - self._last_refresh) < self._min_interval:
            return
        self._last_refresh = now
        try:
            self.dialog.progressValue = min(self.done, self.total)
            self.dialog.message = self._format(label)
            self._do_events()
        except Exception:
            pass

    def _eta_seconds(self):
        """Estimate the remaining time in seconds, or None while warming up.

        Uses the throughput over a recent sliding window (responsive and already
        inclusive of periodic heavy phases), falling back to the whole-run
        average before the window has data, then smooths the result with an EMA
        so the displayed number doesn't jump around.
        """
        remaining = self.total - self.done
        if remaining <= 0:
            return 0.0

        elapsed = time.time() - self.start_time if self.start_time else 0.0
        if self.done <= 0 or elapsed < self._eta_warmup:
            return None  # not enough data yet

        rate = None  # items per second
        if len(self._samples) >= 2:
            t0, d0 = self._samples[0]
            t1, d1 = self._samples[-1]
            if t1 > t0 and d1 > d0:
                rate = (d1 - d0) / (t1 - t0)
        if not rate:  # fall back to the cumulative average
            rate = self.done / elapsed if elapsed > 0 else None
        if not rate:
            return None

        raw = remaining / rate
        if self._eta_ema is None:
            self._eta_ema = raw
        else:
            self._eta_ema = self._eta_alpha * raw + (1 - self._eta_alpha) * self._eta_ema
        return self._eta_ema

    def _format(self, label):
        done = min(self.done, self.total)
        elapsed = time.time() - self.start_time if self.start_time else 0.0
        pct = int(100 * done / self.total)

        eta_s = self._eta_seconds()
        if eta_s is None:
            eta = 'estimating...'
        else:
            eta = self._fmt_time(eta_s)

        lines = []
        header = self._phase
        if label:
            header = f'{self._phase}: {label}' if self._phase else label
        if header:
            lines.append(header)
        lines.append(f'{done} / {self.total} {self.unit} ({pct}%)')
        lines.append(f'Elapsed: {self._fmt_time(elapsed)}   |   Remaining: {eta}')
        return '\n'.join(lines)

    @staticmethod
    def _fmt_time(seconds):
        seconds = int(round(seconds))
        if seconds < 0:
            seconds = 0
        m, s = divmod(seconds, 60)
        h, m = divmod(m, 60)
        if h:
            return f'{h}h {m:02d}m'
        if m:
            return f'{m}m {s:02d}s'
        return f'{s}s'

    @staticmethod
    def _do_events():
        # Let Fusion repaint the dialog and process the Cancel click.
        if adsk is not None:
            try:
                adsk.doEvents()
            except Exception:
                pass


def count_link_occurrences(root):
    """
    Count how many occurrences will become links, mirroring the filter logic in
    urdf_*/link.py:make_inertial_dict (skip sensors and pure subassembly
    containers). Used to size the progress bar for component mode.
    """
    from . import sensors as core_sensors

    count = 0

    def rec(occurrences):
        nonlocal count
        for occ in occurrences:
            try:
                if core_sensors.is_sensor_occurrence(occ):
                    continue
            except Exception:
                pass
            try:
                has_bodies = occ.bRepBodies.count > 0
                has_children = occ.childOccurrences.count > 0
            except Exception:
                has_bodies, has_children = True, False
            if not has_bodies and has_children:
                rec(occ.childOccurrences)
                continue
            count += 1
            if has_children:
                rec(occ.childOccurrences)

    try:
        rec(root.occurrences)
    except Exception:
        pass
    return count


def _occurrence_own_body_count(occ):
    """Bodies that an occurrence contributes directly (its own bRepBodies, or the
    linked component's nativeObject bodies). Mirrors the body loop in
    sdf.py:add_link / core/mesh.py."""
    try:
        n = occ.bRepBodies.count
    except Exception:
        n = 0
    if n == 0:
        try:
            if getattr(occ, 'isReferencedComponent', False) and getattr(occ, 'nativeObject', None):
                n = occ.nativeObject.bRepBodies.count
        except Exception:
            n = 0
    return n


def count_component_bodies(root):
    """
    Total number of bodies that will be exported in SDF component mode: the sum
    of every non-sensor occurrence's own bodies (recursing into subassemblies).
    """
    from . import sensors as core_sensors

    total = 0

    def rec(occurrences):
        nonlocal total
        for occ in occurrences:
            try:
                if core_sensors.is_sensor_occurrence(occ):
                    continue
            except Exception:
                pass
            total += _occurrence_own_body_count(occ)
            try:
                if occ.childOccurrences.count > 0:
                    rec(occ.childOccurrences)
            except Exception:
                pass

    try:
        rec(root.occurrences)
    except Exception:
        pass
    return total
