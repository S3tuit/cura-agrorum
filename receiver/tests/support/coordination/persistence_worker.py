"""Bounded exception propagation for actual persistence-worker component tests."""

from cura_receiver.persistence_worker import PersistenceWorker


class CheckedPersistenceWorker(PersistenceWorker):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.failure = None

    def run(self):
        try:
            super().run()
        except BaseException as error:
            self.failure = error

    def finish_test(self):
        self.request_stop(deadline_monotonic_us=0)
        self.join(5)
        assert not self.is_alive(), "persistence worker did not reach a safe exit"
        if self.failure is not None:
            raise self.failure


def prepare_worker_files(tmp_path):
    import json
    from cura_receiver.database_initializer import initialize_database
    from tests.support.builders.persistence import GROUP

    database = tmp_path / "worker.db"
    initialize_database(database, GROUP)
    configuration = tmp_path / "test-group.json"
    configuration.write_text(
        json.dumps(
            {
                "format_version": 1,
                "group_id": GROUP.hex(),
                "group_master_key": "00" * 32,
                "active_node_ids": [],
                "retired_node_ids": [],
            }
        )
    )
    configuration.chmod(0o600)
    boot = tmp_path / "boot-id"
    boot.write_text("00112233-4455-6677-8899-aabbccddeeff\n")
    return database, configuration, boot
