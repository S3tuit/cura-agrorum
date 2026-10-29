"""Small primitive oracle for durable effects and volatile ownership.

No receiver algorithm, codec, builder or binder participates in expectations.
The oracle deliberately uses copied dictionaries and a list of input records.
"""

from copy import deepcopy
from dataclasses import dataclass


@dataclass(frozen=True)
class Work:
    uid: int
    instance: bytes
    sequence: int
    kind: str
    value: int = 10
    node: bytes = b"n" * 8
    message: int = 100
    sample: int = 200
    body: bytes = b""
    frame: bytes = b""
    domain: int = 1
    poison: bool = False


class DurableModel:
    def __init__(self):
        self.rows = {"clocks": {}, "profiles": {}, "readings": {}, "quarantine": {}, "messages": {}, "exceptions": {}}
        self.queue = []
        self.batch = []
        self.prepared = {}
        self.isolating = False
        self.seen_poison = set()
        self.ready_poison = set()
        self.unknown = False
        self.state = "AVAILABLE"
        self.claimed = 0

    def restart(self):
        self.queue.clear()
        self.batch.clear()
        self.prepared.clear()
        self.isolating = False
        self.seen_poison.clear()
        self.ready_poison.clear()
        self.unknown = False
        self.state = "AVAILABLE"
        self.claimed = 0

    def _prepare(self, work, view):
        identity = (work.instance, work.sequence)
        if work.kind == "clock":
            return [("clocks", identity, work.value)]
        if work.poison:
            raise ValueError("poison")
        if work.kind == "profile":
            return [("profiles", identity, (3, 0)), ("exceptions", identity, (work.frame, 5))]
        key = (work.node, work.message)
        prior = view["readings"].get(key)
        sample = work.sample if prior is None else prior[0]
        canonical = next(
            (row for (node, _), row in view["readings"].items()
             if node == work.node and row[0] == sample and row[2]), None)
        if prior is not None:
            classification = (
                5 if prior[0] != work.sample or view["messages"][key] != work.domain
                else 4 if prior[1] != work.body or prior[1] != canonical[1] else 2)
            effects = []
        else:
            classification = 1 if canonical is None else 3 if canonical[1] == work.body else 4
            effects = [("messages", key, work.domain),
                       ("readings", key, (work.sample, work.body, int(canonical is None)))]
        if identity in view["profiles"]:
            classification = view["profiles"][identity][1]
        if classification in (4, 5):
            effects.append(("exceptions", identity, (work.frame, 6 if classification == 4 else 7)))
        return [("profiles", identity, (work.domain, classification))] + effects

    def attempt(self, count, fault="none"):
        if self.state == "INCOMPATIBLE" or not self.queue:
            return None
        if not self.batch:
            self.batch = list(self.queue[:count])
        self.claimed = len(self.batch)
        selected = self.batch[:]
        if self.isolating:
            selected = selected[:1]
        if fault == "begin":
            self.state = "IO"
            if not self.unknown and not self.isolating:
                self.claimed = 0
            return "unknown" if self.unknown else "failed"
        view = deepcopy(self.rows)
        try:
            for work in selected:
                if work.uid in self.ready_poison:
                    effects = [("quarantine", work.uid, (work.instance, work.sequence))]
                else:
                    if work.uid not in self.prepared:
                        self.prepared[work.uid] = self._prepare(work, view)
                    effects = self.prepared[work.uid]
                for table, key, value in effects:
                    if key in view[table] and view[table][key] != value:
                        raise KeyError("identity collision")
                    view[table][key] = value
        except ValueError:
            if self.isolating and work.uid in self.seen_poison:
                self.ready_poison.add(work.uid)
            self.seen_poison.add(work.uid)
            self.isolating = True
            return "failed"
        except KeyError:
            self.state = "INCOMPATIBLE"
            return "failed"
        if fault == "commit_absent":
            self.state = "IO"
            self.unknown = True
            return "unknown"
        self.rows = view
        if fault == "commit_durable":
            self.state = "IO"
            self.unknown = True
            return "unknown"
        self.unknown = False
        del self.queue[: len(selected)]
        del self.batch[: len(selected)]
        self.claimed = len(self.batch)
        for work in selected:
            self.prepared.pop(work.uid, None)
            self.seen_poison.discard(work.uid)
            self.ready_poison.discard(work.uid)
        if not self.batch:
            self.prepared.clear()
            self.isolating = False
            self.seen_poison.clear()
            self.ready_poison.clear()
            self.claimed = 0
            self.state = "AVAILABLE"
        return "committed"
