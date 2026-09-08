"""Corner callback uses the existing single-owner transport and real cleanup."""
import unittest
import y42_position_microprobe_io as io
from test_y42_torque_admission import TorqueFakeBus


class CornerBus(TorqueFakeBus):
    def assert_c5_contract(self,packet):
        assert len(packet)==11 and packet[1]==0xC5 and packet[7:]==b'\x00\x00\x0a\x6b'
        assert int.from_bytes(packet[3:5],'big')==200
        assert int.from_bytes(packet[5:7],'big') in (20,40,60)
        assert all(int.from_bytes(cfg[16:18],'big')==60 and int.from_bytes(cfg[18:20],'big')==1
                   for cfg in self.configs.values())


class CornerTransportTests(unittest.TestCase):
    def test_position_mode_small_cb_step_and_cleanup(self):
        bus=CornerBus()
        def phase(session,**unused):
            actual=session.read_feedback_43()[0]['position']
            result=session.send_cb(actual+0.1,40)
            self.assertTrue(result['accepted'])
            self.assertEqual(bytes.fromhex(result['tx_hex'])[1],0xCB)
            return {'success':True,'reason':'fake bounded CB'}
        result=io.run_hardware_corner_probe(bus.api(),phase,clock=bus.clock,sleep=bus.clock.sleep,position_mode=True)
        self.assertTrue(result['success'],result['reason'])
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])

    def test_position_mode_refuses_excess_lead_and_active_c5(self):
        for active_c5 in (False,True):
            bus=CornerBus()
            def phase(session,**unused):
                if active_c5:
                    session.send_c5(bytes.fromhex('01c50000c8001400000a6b'))
                else:
                    actual=session.read_feedback_43()[0]['position']
                    session.send_cb(actual+0.3,40)
            result=io.run_hardware_corner_probe(bus.api(),phase,clock=bus.clock,sleep=bus.clock.sleep,position_mode=True)
            self.assertFalse(result['success'])
            self.assertFalse(bus.c5_packets)
            self.assertTrue(result['final_disabled_confirmed'])

    def test_unsolicited_post_seed_error_stops_before_corner_motion(self):
        bus=CornerBus();api=bus.api();original=api.direct.transact
        def transact(connection,packet,**kwargs):
            reply=original(connection,packet,**kwargs)
            if connection.phase=='zero_current_cb_seed' and packet[0]==1:
                bus.rx=bytes.fromhex('0100ee6b')
            return reply
        api.direct.transact=transact
        result=io.run_hardware_corner_probe(api,lambda session,**kw:{'success':True},clock=bus.clock,sleep=bus.clock.sleep)
        self.assertFalse(result['success'])
        self.assertIn('0100ee6b',result['reason'])
        self.assertFalse(bus.c5_packets)
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])

    def test_preparation_foreign_error_read_is_archived_and_reissued_once(self):
        bus=CornerBus();api=bus.api();original=api.direct.transact;injected=[]
        def transact(connection,packet,**kwargs):
            reply=original(connection,packet,**kwargs)
            if packet==bytes.fromhex('03437a6b') and connection.phase=='before_global_20' and not injected:
                injected.append(True)
                return reply+bytes.fromhex('0100ee6b')
            return reply
        api.direct.transact=transact
        result=io.run_hardware_corner_probe(api,lambda session,**kw:{'success':True,'reason':'fake'},clock=bus.clock,sleep=bus.clock.sleep)
        self.assertTrue(result['success'],result['reason'])
        self.assertEqual(result['preparation_read_recoveries'],1)
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])

    def test_repeated_preparation_error_remains_failure(self):
        bus=CornerBus();api=bus.api();original=api.direct.transact
        def transact(connection,packet,**kwargs):
            reply=original(connection,packet,**kwargs)
            if packet==bytes.fromhex('03437a6b') and connection.phase.startswith('before_global_20'):
                return reply+bytes.fromhex('0100ee6b')
            return reply
        api.direct.transact=transact
        result=io.run_hardware_corner_probe(api,lambda session,**kw:{'success':True},clock=bus.clock,sleep=bus.clock.sleep)
        self.assertFalse(result['success'])
        self.assertEqual(result['preparation_read_recoveries'],1)
        self.assertFalse(bus.c5_packets)
        self.assertTrue(result['final_disabled_confirmed'])

    def test_uncertain_configuration_write_is_not_reissued(self):
        bus=CornerBus();api=bus.api();original=api.direct.transact;writes=[]
        def transact(connection,packet,**kwargs):
            reply=original(connection,packet,**kwargs)
            if connection.phase=='global_20_write':
                writes.append(bytes(packet))
                return reply+bytes.fromhex('0100ee6b')
            return reply
        api.direct.transact=transact
        result=io.run_hardware_corner_probe(api,lambda session,**kw:{'success':True},clock=bus.clock,sleep=bus.clock.sleep)
        self.assertFalse(result['success'])
        self.assertEqual(len(writes),1)
        self.assertEqual(result['preparation_read_recoveries'],0)
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])

    def test_owned_phase_and_cleanup(self):
        bus=CornerBus()
        def phase(session,**unused):
            self.assertEqual(session.corner_probe_axis_id,1)
            session.send_c5(bytes.fromhex('01c50000c8003c00000a6b'))
            feedback=session.read_feedback_43()
            self.assertEqual(len(feedback),4)
            self.assertTrue(all(row.get('raw_packet_hex') for row in feedback))
            return {'success':True,'reason':'bounded fake phase','candidate_stop':False}
        result=io.run_hardware_corner_probe(bus.api(),phase,clock=bus.clock,sleep=bus.clock.sleep)
        self.assertTrue(result['success'],result['reason'])
        self.assertEqual(result['stage'],'corner_stop_probe')
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])
        self.assertEqual(bus.configs,bus.original)
        self.assertFalse(any(cap==150 for _,_,cap,_ in bus.config_history))

    def test_invalid_phase_command_rejected_before_c5_and_cleanup_runs(self):
        bus=CornerBus()
        def phase(session,**unused):
            session.send_c5(bytes.fromhex('01c50000c8005000000a6b'))
        result=io.run_hardware_corner_probe(bus.api(),phase,clock=bus.clock,sleep=bus.clock.sleep)
        self.assertFalse(result['success'])
        self.assertFalse(bus.c5_packets)
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])

    def test_phase_exception_does_not_leave_torque_active(self):
        bus=CornerBus()
        def phase(session,**unused):
            session.send_c5(bytes.fromhex('01c50000c8001400000a6b'))
            raise RuntimeError('injected phase failure')
        result=io.run_hardware_corner_probe(bus.api(),phase,clock=bus.clock,sleep=bus.clock.sleep)
        self.assertFalse(result['success'])
        self.assertTrue(result['final_disabled_confirmed'])
        self.assertTrue(result['configuration_restored'])


if __name__=='__main__':unittest.main()
