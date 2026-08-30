from pathlib import Path
import tempfile
import unittest

from factory_core.station_config import load_station_definitions


class StationConfigTests(unittest.TestCase):
    def test_loads_navigation_and_perception_data_together(self):
        config = Path(__file__).parents[1] / "config" / "stations.yaml"
        stations = load_station_definitions(config)

        self.assertEqual(stations["raw_bin"].tag_id, 10)
        self.assertEqual(stations["charge_dock"].station_type, "charging")
        self.assertAlmostEqual(stations["machine_2"].dock_pose.y, 1.98)
        self.assertAlmostEqual(
            stations["charge_dock"].dock_pose.y, -3.087
        )

    def test_rejects_duplicate_tag_ids(self):
        document = (
            "stations:\n"
            "  first:\n"
            "    type: non_charging\n"
            "    tag_id: 10\n"
            "    staging_pose: [0, 0, 0]\n"
            "    dock_pose: [0, 0, 0]\n"
            "  second:\n"
            "    type: non_charging\n"
            "    tag_id: 10\n"
            "    staging_pose: [1, 0, 0]\n"
            "    dock_pose: [1, 0, 0]\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "stations.yaml"
            config.write_text(document, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "reuses AprilTag ID"):
                load_station_definitions(config)


if __name__ == "__main__":
    unittest.main()
