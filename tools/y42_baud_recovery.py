"""Bounded, disabled-only recovery after an unsuccessful baud change."""


def restore_baud_snapshot(configs, old_baud, attempted_baud, exchange, attempts=5):
    """exchange(baud, tx) owns serialization and records every raw transaction."""
    results = []
    for address, original in sorted(configs.items()):
        if len(original) != 37 or original[:4] != bytes((address, 0x42, 0x25, 0x18)) or original[-1] != 0x6B:
            raise ValueError("Invalid original configuration snapshot")
    for address, original in sorted(configs.items()):
        query = bytes((address, 0x42, 0x6C, 0x6B))
        status = bytes((address, 0x3A, 0x6B))
        expected_disabled = bytes((address, 0x3A, 2, 0x6B))
        verified = False
        for _ in range(attempts):
            if exchange(old_baud, query) == original:
                verified = exchange(old_baud, status) == expected_disabled
                if verified:
                    break
                continue
            # No configuration write unless this axis is freshly confirmed
            # disabled. A corrupt status frame never authorizes the write.
            disabled = any(exchange(attempted_baud, status) == expected_disabled for _ in range(8))
            if not disabled:
                break
            exchange(attempted_baud, bytes((address, 0x48, 0xD1, 1)) + original[4:])
        if not verified:
            verified = exchange(old_baud, query) == original and exchange(old_baud, status) == expected_disabled
        results.append({"id": address, "restored": verified})
    return {"restored": bool(results) and all(row["restored"] for row in results), "results": results}
