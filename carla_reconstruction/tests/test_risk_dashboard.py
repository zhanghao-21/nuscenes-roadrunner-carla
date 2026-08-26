import unittest

from closed_loop.risk_dashboard import (
    _actor_plot_text,
    _compact_actor_id,
    _dashboard_status_text,
    _dashboard_window_size,
)


class DashboardLayoutTests(unittest.TestCase):
    def test_default_two_column_window_fits_known_screen(self):
        self.assertEqual(
            _dashboard_window_size(450, 3, 1920, 1040),
            (940, 465))

    def test_window_size_is_clamped_to_available_screen(self):
        self.assertEqual(
            _dashboard_window_size(450, 3, 800, 500),
            (760, 450))

    def test_live_titles_stay_compact_and_full_details_move_to_tooltip(self):
        actor_id = "sumo:nusc_c283b224a9984736bff67a2f347866fa"
        status = "Live HGT prediction; one vehicle warming"
        actor = {
            "actor_id": actor_id,
            "evaluated_mode_count": 20,
            "worst_inverse_ttc_s_inv": 10.0,
            "expected_inverse_ttc_s_inv": 8.0,
        }

        bar_title, history_title, tooltip = _actor_plot_text(
            actor, "12.3 m", "ready", status)

        self.assertEqual(bar_title.count("<br>"), 2)
        self.assertEqual(history_title.count("<br>"), 1)
        self.assertNotIn(actor_id, bar_title)
        self.assertNotIn(actor_id, history_title)
        self.assertNotIn(status, bar_title)
        self.assertIn(actor_id, tooltip)
        self.assertIn(status, tooltip)
        self.assertLessEqual(len(_compact_actor_id(actor_id)), 24)
        self.assertTrue(_compact_actor_id(actor_id).startswith("sumo:"))

    def test_live_header_uses_two_bounded_lines(self):
        status = "Warming trajectory histories (0/3 ready)"
        text = _dashboard_status_text(12.5, status, 3)
        first_line, second_line = text.split("<br>")
        self.assertIn("Simulation time: 12.50 s", first_line)
        self.assertIn("Active actors: 3", first_line)
        self.assertEqual(second_line, "Status: " + status)


if __name__ == "__main__":
    unittest.main()
