import unittest
from y42_position_microprobe_io import audit_sync_tail


class TailAudit(unittest.TestCase):
    def test_real_archived_reply_keeps_body_and_error_separate_but_rejected(self):
        tx=bytes.fromhex('03356b')
        reply=bytes.fromhex('03350000006b0100ee6b')
        audit=audit_sync_tail(tx,reply)
        self.assertEqual(audit['body_hex'],'03350000006b')
        self.assertEqual(audit['tail_hex'],'0100ee6b')
        self.assertEqual(audit['error_axis'],1)
        self.assertFalse(audit['accepted'])
        self.assertEqual(bytes.fromhex(audit['body_hex']+audit['tail_hex']),reply)

    def test_plain_reply_and_malformed_body_are_not_mislabeled_extra_packets(self):
        tx=bytes.fromhex('03356b')
        self.assertIsNone(audit_sync_tail(tx,bytes.fromhex('03350000006b')))
        self.assertIsNone(audit_sync_tail(tx,bytes.fromhex('0335000000000100ee6b')))


if __name__=='__main__':unittest.main()
