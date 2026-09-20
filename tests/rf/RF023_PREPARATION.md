# RF-023 local packet preparation

Implemented: `rejection_vectors.py` constructs the finite packet matrix and independently expected ACK bytes/profile fields. Host tests send these bytes through the real `ProtocolIngress` with available production queue admission. The packet builder uses literal wire layout and AES-CCM independently of the production frame encoder/ACK builder.

The existing C6 radio test app also implements twelve named cases using production crypto/radio, one TX per command and a 500 ms ACK/silence window from TX_DONE. Host checks compare its actual C builder with the independent Python vectors. Ordinary and enabled images are built locally only.

Not implemented or qualified by this component: installed-service orchestration/revocation restart, physical ACK/silence observation, database/journal reconciliation, or full RF-023 execution. The RF-023 catalogue remains unchecked. Review the integrated session mechanism before device execution; RF-020 remains its prerequisite. Do not transmit these vectors through the ordinary production node image or reuse its identity/counters.

## Private local build inputs

Using the repository's receiver Python environment, run `tests/rf/prepare_rejection_inputs.py --output /tmp/<new-private-directory> --run <32-lowercase-hex-run-id>`. The directory must not exist. Preparation generates a new disposable group and three identities using production provisioning primitives; it never reads the production credentials. Both before/after receiver allowlists and the generated C header are private. The public manifest records source/header hashes, fixed packet/ACK bytes and each short `command_case` alias.

Build locally with the configured ESP-IDF environment:

```sh
CCACHE_DISABLE=1 idf.py -C firmware/test_apps/radio -B /tmp/<private-build-directory> \
  -D RF023_INPUT_DIR=/tmp/<new-private-directory> build
```

This does not flash or run anything. Enabled images contain private credentials and must not be placed in public evidence. The build rejects changed bound sources or headers; prepare a fresh bundle instead of editing the manifest to bypass this check. UART commands use the prepared run ID, the manifest's alias, phase zero and the current boot nonce. Matrix allowlist phases are separate from the UART phase field. No integrated hardware runner is supplied yet.

## Matrix

| Case | Expected first decision | Selected ACK |
| --- | --- | --- |
| implausible_reading | ACCEPTED; preserve representable values unchanged | domain03/status0 |
| unsupported_control | REJECTED_UNSUPPORTED_CONTROL | domain05/status2 |
| unknown_domain | REJECTED_UNSUPPORTED_DOMAIN | domain05/status2 |
| malformed_length | Authenticated short reading body: REJECTED_MALFORMED_LENGTH | domain06/status3 |
| malformed_flags | Nonzero soil0 with its validity bit cleared: REJECTED_MALFORMED_BODY | domain06/status3 |
| wrong_direction | Valid control and ACK downlink domain: WRONG_DIRECTION | none |
| unsupported_control_wrong_direction | Unsupported control precedes direction: REJECTED_UNSUPPORTED_CONTROL | domain05/status2 |
| bad_tag | AUTHENTICATION_FAILED | none |
| unknown_node | UNKNOWN_NODE | none |
| short_header | REJECTED_MALFORMED_LENGTH; expose only length-valid untrusted claims | none |
| revocation_baseline | ACCEPTED for the identity that will be revoked | domain03/status0 |
| revoked_node | UNKNOWN_NODE after the explicit isolated-service allowlist change/restart | none |

The first eleven vectors belong to the original allowlist; the last belongs to the post-revocation phase. The baseline and revoked message have distinct counters. Future integrated execution must preserve the baseline's historical database rows through revocation and distinguish this real transition from a node that was always unknown.

## Identity and nonce boundary

`matrix(active=(node_id, key), unknown=(node_id, key), revoked=(node_id, key), first_message=..., first_sample=...)` requires three distinct disposable identities. The caller must provision a new dedicated test identity set and reserve the full12-counter range for this matrix; this pure builder is not an allocator or durable ledger. It rejects aliased node IDs and counter overflow. It returns immutable vector records containing frames/expectations, never keys.

Each changed authenticated message uses a distinct message counter. Uplink/downlink domains remain part of the nonce. The bad-tag case encrypts once and corrupts that frame's tag; it does not create a second authenticated plaintext for the same nonce. No transport-conflict vector is generated. Host fixtures use public synthetic keys; those are not deployment credentials.

## Observation boundary

`verify_profile(vector, profile)` checks exact padded received bytes/length, length-valid header claims, authentication/decode exposure, processing result and selected ACK bytes against a SQLite-shaped profile mapping. A passing profile check proves neither ACK transmission nor reception. In particular, an absent selected ACK is not measured RF silence.

The future hardware runner must separately establish the bounded transmit/observe episode, real C6 reception or timeout, actual Pi TX trace/profile completion, durable canonical reading/body, no synthetic diagnostics for ordinary protocol outcomes, explicit service revocation lifecycle and cleanup. No automatic retries or hardware admission are supplied by this module.

Validation: `make test-rf-host`. The dedicated host tests cover all12 cases against production ingress, independent nonce/decryption checks, counter/identity guards and rejection of mismatched observation fields. Hardware qualification remains NOT RUN.
