from pathlib import Path
import tempfile
import unittest

from factory_core.route_config import load_factory_routes


class RouteConfigTests(unittest.TestCase):
    def test_loads_south_transit_lane(self):
        config = Path(__file__).parents[1] / "config" / "routes.yaml"
        routes = load_factory_routes(config)

        route = routes["raw_bin_to_finished_bin"]
        self.assertEqual(len(route.waypoints), 3)
        self.assertAlmostEqual(route.waypoints[0].y, -1.70)
        self.assertAlmostEqual(route.waypoints[1].x, 1.50)
        self.assertAlmostEqual(route.waypoints[2].y, -2.70)

    def test_rejects_empty_route(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "routes.yaml"
            config.write_text(
                "routes:\n  empty:\n    waypoints: []\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "at least one waypoint"):
                load_factory_routes(config)

    def test_rejects_non_finite_waypoint(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "routes.yaml"
            config.write_text(
                "routes:\n  invalid:\n    waypoints: [[.nan, 0, 0]]\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "must be finite"):
                load_factory_routes(config)


if __name__ == "__main__":
    unittest.main()
