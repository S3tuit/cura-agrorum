"""Create fresh private RF-023 build inputs; never read existing production credentials."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import re
import sys

from evidence import REPO, digest
sys.path[:0] = [str(REPO / 'receiver'), str(REPO / 'protocol/protocol-v2-lora/python'),
               str(REPO / 'protocol/protocol-v2-lora/tools')]
from provisioning_common import (generate_receiver_group, generate_unique_node_id,
    derive_node_key, render_receiver_group, write_new_secret)
from rejection_vectors import matrix, C6_CASES

BOUND_SOURCES = (
    'firmware/test_apps/radio/main/radio_rejection.c',
    'firmware/test_apps/radio/main/radio_rejection.h',
    'firmware/test_apps/radio/main/radio_app.c',
    'firmware/test_apps/radio/main/radio_command.c',
    'firmware/test_apps/radio/main/radio_command.h',
    'firmware/components/protocol_v2_lora/protocol_v2_lora_crypto.c',
    'firmware/components/protocol_v2_lora/protocol_v2_lora_schema_generated.c',
    'firmware/components/protocol_v2_lora/include/protocol_v2_lora_crypto.h',
    'firmware/components/protocol_v2_lora/include/protocol_v2_lora_schema_generated.h',
)


def prepare(output, run):
    if not re.fullmatch('[0-9a-f]{32}', run):
        raise ValueError('run must be32 lowercase hex characters')
    output = Path(output).resolve()
    if output.is_relative_to(REPO) and not output.is_relative_to(REPO / 'tests/rf/raw'):
        raise ValueError('private inputs must be outside the tree or in ignored tests/rf/raw')
    output.mkdir(mode=0o700)  # Refuse reuse/overwrite, including old keys/counters.
    group = generate_receiver_group()
    identities = []
    for _ in range(3):
        node = generate_unique_node_id(group)
        identities.append((node, derive_node_key(group.group_master_key, node)))
        group = replace(group, active_node_ids=group.active_node_ids | {node})
    active, unknown, revoked = identities
    before = replace(group, active_node_ids=frozenset((active[0], revoked[0])))
    after = replace(before, active_node_ids=frozenset((active[0],)), retired_node_ids=frozenset((revoked[0],)))
    write_new_secret(output / 'receiver-group-before.json', render_receiver_group(before))
    write_new_secret(output / 'receiver-group-after.json', render_receiver_group(after))
    def array(value):
        return '{' + ','.join('0x%02x' % b for b in value) + '}'
    header = ('/* PRIVATE: disposable RF-023 credentials; never archive this file or enabled images. */\n'
              '#pragma once\n#include "radio_rejection.h"\n'
              'static const rf023_config_t rf023_config = {\n'
              f' .run = "{run}",\n'
              ' .node_ids = {' + ','.join(array(n) for n, _ in identities) + '},\n'
              ' .node_keys = {' + ','.join(array(k) for _, k in identities) + '},\n'
              ' .first_message = 0, .first_sample = 0\n};\n')
    write_new_secret(output / 'rf023_inputs.h', header)
    vectors = matrix(active=active, unknown=unknown, revoked=revoked, first_message=0, first_sample=0)
    public = dict(schema=1, run=run, group_id=group.group_id.hex(),
                  nodes=dict(active=active[0].hex(), unknown=unknown[0].hex(), revoked=revoked[0].hex()),
                  sources={name: digest(REPO / name) for name in BOUND_SOURCES},
                  input_header_sha256=digest(output / 'rf023_inputs.h'),
                  first_message=0, first_sample=0,
                  cases=[dict(name=v.name, command_case=case, phase=v.phase, frame=v.frame.hex(), ack=v.ack.hex() if v.ack else None,
                              processing_result=v.processing_result, sample_id=v.sample_id)
                         for v, case in zip(vectors, C6_CASES, strict=True)])
    # Header/keys and both allowlists are private; this public manifest has only hashes/frames/IDs.
    with (output / 'rf023-manifest.json').open('x') as stream:
        json.dump(public, stream, indent=2, sort_keys=True); stream.write('\n')
    return public


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', required=True)
    args = parser.parse_args()
    value = prepare(args.output, args.run)
    print(json.dumps(dict(run=value['run'], group_id=value['group_id'], manifest=str(args.output / 'rf023-manifest.json'))))


if __name__ == '__main__':
    main()
