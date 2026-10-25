"""The generated Python messages, pinned to the same ICD bit patterns as the C++ tests."""

import unittest

from fmofp_bus import word as w
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import ControlSurface, Ssm, TerminalState, WeatherMode
from fmofp_bus.status import Status


class GeneratedMessagesTest(unittest.TestCase):
    def test_version_and_table(self):
        self.assertEqual(1, m.ICD_MAJOR)
        self.assertEqual(4, m.ICD_MINOR)
        self.assertEqual(14, len(m.RELAYS))
        self.assertEqual(("PLANT_SENSORS", "FCS_SENSORS", 27, 1, 5, 13, 11, 50), tuple(m.RELAYS[0]))
        self.assertEqual(66, len(m.MESSAGES))
        self.assertIs(m.FcsAttitude, m.MESSAGE_CLASSES["FCS_ATTITUDE"])
        self.assertEqual(m.EVERY_TERMINAL, m.TermStatus.RT)

    def test_fcs_attitude_lands_where_the_table_says(self):
        msg = m.FcsAttitude(roll_ssm=Ssm.NORMAL, pitch_ssm=Ssm.NORMAL, yaw_ssm=Ssm.NORMAL, roll=45.0, pitch=-90.0)
        self.assertEqual((Status.OK, [0xFC00, 0x2000, 0xC000, 0x0000]), msg.encode())
        self.assertEqual((5, 12, "T", 4, 50), (m.FcsAttitude.RT, m.FcsAttitude.SUBADDRESS, m.FcsAttitude.DIRECTION,
                                               m.FcsAttitude.WORDS, m.FcsAttitude.RATE_HZ))

    def test_out_of_range_is_clamped_and_reported(self):
        status, words = m.FcsAttitude(roll=200.0).encode()
        self.assertEqual(Status.CLAMPED, status)
        self.assertEqual(0x7FFF, words[1])

    def test_defaults_are_fail_safe(self):
        status, words = m.FcsAttitude().encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual([0, 0, 0, 0], words)
        self.assertEqual(Ssm.FAILURE_WARNING, m.FcsAttitude().roll_ssm)

    def test_term_status(self):
        msg = m.TermStatus(state=TerminalState.INITIALIZING, last_cmd_seq=0x42, healthy=True, icd_major=1)
        status, words = msg.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual([0xF042, 0x0001, 0x0000, 0x0100, 0x0000], words)
        status, back = m.TermStatus.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual(msg, back)

    def test_undefined_enum_is_rejected_both_ways(self):
        words = [(1 << 12) | (9 << 8), 0]
        self.assertEqual((Status.INVALID_ENUM, None), m.RdrWxStatus.decode(words))
        words = [(1 << 12) | (1 << 8), 0]
        status, back = m.RdrWxStatus.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual(WeatherMode.SURVEILLANCE, back.mode)
        self.assertEqual(Status.INVALID_ARGUMENT, m.RdrWxStatus(mode=9).encode()[0])
        self.assertEqual((Status.BAD_LENGTH, None), m.RdrWxStatus.decode([0]))

    def test_surface_command_in_hundredths_of_percent(self):
        msg = m.FcsSurfaceCmd(surface=ControlSurface.ELEVATOR, cmd_seq=3, position=-12.34)
        status, words = msg.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual([0x0203, w.to_twos(-1234, 16)], words)
        status, back = m.FcsSurfaceCmd.decode(words)
        self.assertAlmostEqual(-12.34, back.position)

    def test_nav_update_present_bits_and_unsigned_scales(self):
        msg = m.FmsNavUpdateCmd(altitude_present=True, heading_present=True, cmd_seq=0xAB, altitude=35000.0,
                                heading=270.0)
        self.assertEqual((Status.OK, [0xA0AB, 35000, 0, 0xC000, 0]), msg.encode())

    def test_layout_of_shares_words_not_address(self):
        cmd = m.MfdRdrWxCmd()
        cmd.payload.requested_mode = WeatherMode.PRECIPITATION
        cmd.payload.cmd_seq = 9
        cmd.payload.range_km = 80.0
        cmd.payload.tilt = 2.5
        status, words = cmd.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual((13, 20, "T", 10), (m.MfdRdrWxCmd.RT, m.MfdRdrWxCmd.SUBADDRESS, m.MfdRdrWxCmd.DIRECTION,
                                             m.MfdRdrWxCmd.RATE_HZ))
        self.assertEqual((9, "R"), (m.RdrWxModeCmd.RT, m.RdrWxModeCmd.DIRECTION))
        status, as_radar = m.RdrWxModeCmd.decode(words)
        self.assertEqual(9, as_radar.cmd_seq)
        self.assertEqual(80.0, as_radar.range_km)
        self.assertEqual(Status.CLAMPED, m.RdrWxModeCmd().encode()[0])  # range_km min is 10

    def test_every_message_round_trips_and_decodes_zero_words(self):
        for info in m.MESSAGES:
            with self.subTest(message=info.id):
                status, back = info.cls.decode([0] * info.words)
                self.assertEqual(Status.OK, status)
                status, words = back.encode()
                self.assertIn(status, (Status.OK, Status.CLAMPED))
                self.assertEqual(info.words, len(words))
                status, again = info.cls.decode(words)
                self.assertEqual(Status.OK, status)
                self.assertEqual(words, again.encode()[1])


if __name__ == "__main__":
    unittest.main()
