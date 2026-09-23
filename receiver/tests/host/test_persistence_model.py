from tests.support.models.ordinary_persistence import DurableModel, Work


# Reviewed examples establish commit certainty and retention independently of production code.
def test_model_commit_certainty():
    model = DurableModel()
    work = Work(1, b"a" * 16, 1, "clock")
    model.queue.append(work)
    assert model.attempt(1, "commit_absent") == "unknown"
    assert model.rows["clocks"] == {}
    assert model.claimed == 1 and model.queue == [work]
    assert model.attempt(1, "commit_durable") == "unknown"
    assert model.rows["clocks"] == {(b"a" * 16, 1): 10}
    assert model.queue == [work]
    assert model.attempt(1) == "committed"
    assert model.queue == [] and model.claimed == 0


# Reviewed FIFO examples remove each durably completed prefix immediately.
def test_model_poison_and_valid_prefix():
    model = DurableModel()
    model.queue.extend(
        [Work(1, b"a" * 16, 1, "clock"), Work(2, b"a" * 16, 2, "profile", poison=True)]
    )
    assert model.attempt(2) == "failed"
    assert model.rows["clocks"] == {}
    assert model.attempt(2) == "committed"
    assert model.claimed == 1 and len(model.queue) == 1
    assert model.attempt(2) == "failed"
    assert model.rows["quarantine"] == {}
    assert model.attempt(2) == "committed"
    assert model.rows["quarantine"] == {2: (b"a" * 16, 2)}
    assert model.queue == []


# Literal frames/bodies establish each reading classification and immutable canonical ownership.
def test_model_reading_examples():
    model = DurableModel()
    frames = [b"A", b"A", b"B", b"C", b"D", b"E"]
    bodies = [b"body", b"body", b"body", b"body", b"changed", b"new-sample"]
    messages = [100, 100, 100, 101, 102, 100]
    for index, (frame, body, message) in enumerate(zip(frames, bodies, messages), 1):
        model.queue.append(
            Work(
                index,
                b"a" * 16,
                index,
                "reading",
                message=message,
                sample=201 if index == 6 else 200,
                frame=frame,
                body=body,
            )
        )
    assert model.attempt(6) == "committed"
    assert [value[1] for value in model.rows["profiles"].values()] == [1, 2, 4, 3, 4, 5]
    assert model.rows["readings"][(b"n" * 8, 100)] == (200, b"body", 1, b"a" * 16, 1)
    assert len(model.rows["readings"]) == 3
