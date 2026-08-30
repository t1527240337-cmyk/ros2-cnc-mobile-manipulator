from pathlib import Path
import tempfile
import unittest

from factory_core.physical_campaign import (
    CAMPAIGN_SCHEMA_VERSION,
    _acquire_campaign_lock,
    _evidence_manifest,
    _scenario_digest,
    _scenario_document,
    _validate_result_record,
    parse_raw_targets,
    campaign_summary,
    parse_physical_summary,
    parse_seed_spec,
    scenario_for_seed,
    validate_summary_for_scenario,
)


class SeedParsingTests(unittest.TestCase):
    def test_campaign_result_directory_has_one_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            result_root = Path(directory)
            first = _acquire_campaign_lock(result_root)
            try:
                with self.assertRaisesRegex(
                    RuntimeError, "another physical seed campaign"
                ):
                    _acquire_campaign_lock(result_root)
            finally:
                first.close()

            after_release = _acquire_campaign_lock(result_root)
            after_release.close()

    def test_range_and_duplicates_become_sorted_unique_seeds(self):
        self.assertEqual(
            parse_seed_spec("105,101-103,102"),
            (101, 102, 103, 105),
        )

    def test_descending_range_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "descending"):
            parse_seed_spec("5-3")


class ScenarioTests(unittest.TestCase):
    def test_regular_seeds_rotate_across_all_machines(self):
        self.assertEqual(
            scenario_for_seed(1, "production").allowed_machines,
            ("machine_1",),
        )
        self.assertEqual(
            scenario_for_seed(2, "production").allowed_machines,
            ("machine_2",),
        )
        self.assertEqual(
            scenario_for_seed(3, "production").allowed_machines,
            ("machine_3",),
        )

    def test_seed_deterministically_perturbs_a_bounded_initial_pose(self):
        first = scenario_for_seed(1, "production")
        repeated = scenario_for_seed(1, "production")
        another = scenario_for_seed(2, "production")

        self.assertEqual(
            (first.robot_x, first.robot_y, first.robot_yaw),
            (repeated.robot_x, repeated.robot_y, repeated.robot_yaw),
        )
        self.assertNotEqual(
            (first.robot_x, first.robot_y, first.robot_yaw),
            (another.robot_x, another.robot_y, another.robot_yaw),
        )
        self.assertLessEqual(abs(first.robot_x), 0.06)
        self.assertLessEqual(abs(first.robot_y + 1.2), 0.06)
        self.assertLessEqual(abs(first.robot_yaw), 0.048)

    def test_every_sixth_seed_exercises_repeated_unordered_selection(self):
        scenario = scenario_for_seed(6, "production")

        self.assertEqual(scenario.quantity, 2)
        self.assertGreaterEqual(scenario.raw_part_count, scenario.quantity)
        self.assertEqual(
            scenario.environment(headless=True)["RAW_PART_COUNT"],
            str(scenario.raw_part_count),
        )

    def test_every_fifth_seed_exercises_safe_low_battery_recharge(self):
        scenario = scenario_for_seed(5, "production")

        self.assertEqual(scenario.initial_battery, 0.20)
        self.assertTrue(scenario.expect_recharge)
        self.assertEqual(
            scenario.environment(headless=True)["EXPECT_RECHARGE"],
            "true",
        )

    def test_every_seventh_seed_exercises_empty_machine_fault_reroute(self):
        scenario = scenario_for_seed(7, "production")

        self.assertEqual(scenario.fault_machine_before_order, "machine_1")
        self.assertEqual(scenario.expected_execution_machine, "machine_2")
        self.assertEqual(
            scenario.allowed_machines,
            ("machine_1", "machine_2"),
        )

    def test_three_machine_profile_keeps_one_full_batch(self):
        scenario = scenario_for_seed(101, "three-machine")

        self.assertEqual(scenario.quantity, 3)
        self.assertEqual(scenario.raw_part_count, 6)
        self.assertEqual(
            scenario.allowed_machines,
            ("machine_1", "machine_2", "machine_3"),
        )


class PhysicalSummaryTests(unittest.TestCase):
    def test_accepts_contact_verified_success(self):
        parsed = parse_physical_summary(
            "physical_order_success=true order=seed_1 completed=1 "
            "physics=contact_verified wall_seconds=421"
        )

        self.assertEqual(parsed["completed"], "1")
        self.assertEqual(parsed["wall_seconds"], "421")

    def test_rejects_semantic_only_result(self):
        with self.assertRaisesRegex(ValueError, "contact-verified"):
            parse_physical_summary(
                "physical_order_success=true physics=semantic"
            )

    def test_parses_a_sensor_measured_target_path(self):
        self.assertEqual(
            parse_raw_targets("0.612,-0.311,0.254;0.901,0.287,0.255"),
            ((0.612, -0.311, 0.254), (0.901, 0.287, 0.255)),
        )

    def test_rejects_a_target_outside_the_measured_work_volume(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            parse_raw_targets("2.0,0.0,0.25")

    def test_summary_must_match_the_exact_seed_scenario(self):
        scenario = scenario_for_seed(7, "production")
        fields = parse_physical_summary(
            "physical_order_success=true order=physical_seed_7 completed=1 "
            "machines=machine_1,machine_2 initial_raw_inventory=6 "
            "raw_inventory=5 finished_inventory=1 "
            "orchestration=action physics=contact_verified "
            "fault_isolated=machine_1 raw_source=rgbd_sparse_bin raw_seed=7 "
            "raw_layout=unordered_workspace raw_target_count=1 "
            "raw_targets=0.782,0.000,0.254 "
            "finished_source=rgbd_slots recharge_verified=false "
            f"execution_machine=machine_2 robot_spawn={scenario.robot_x:.3f},"
            f"{scenario.robot_y:.3f},{scenario.robot_yaw:.3f}"
        )

        validate_summary_for_scenario(fields, scenario)
        fields["raw_seed"] = "8"
        with self.assertRaisesRegex(ValueError, "raw_seed"):
            validate_summary_for_scenario(fields, scenario)


class EvidenceIntegrityTests(unittest.TestCase):
    @staticmethod
    def _record(directory: Path, scenario):
        evidence_names = (
            "console.log",
            "physical_order_summary.log",
            "physical_order_truth.log",
            "physical_order_result.log",
            "physical_order_factory_state.log",
            "physical_order_machine_states.log",
        )
        summary = (
            "physical_order_success=true order=physical_seed_1 completed=1 "
            "machines=machine_1 initial_raw_inventory=4 "
            "raw_inventory=3 finished_inventory=1 "
            "orchestration=action physics=contact_verified fault_isolated=none "
            "raw_source=rgbd_sparse_bin raw_seed=1 "
            "raw_layout=unordered_workspace raw_target_count=1 "
            "raw_targets=0.782,0.000,0.254 finished_source=rgbd_slots "
            f"recharge_verified=false execution_machine=machine_1 "
            f"robot_spawn={scenario.robot_x:.3f},{scenario.robot_y:.3f},"
            f"{scenario.robot_yaw:.3f}"
        )
        for name in evidence_names:
            content = summary if name == "physical_order_summary.log" else name
            (directory / name).write_text(content)
        return {
            "schema_version": CAMPAIGN_SCHEMA_VERSION,
            "status": "passed",
            "seed": scenario.seed,
            "profile": scenario.profile,
            "scenario": _scenario_document(scenario),
            "scenario_digest": _scenario_digest(scenario),
            "evidence_manifest": _evidence_manifest(directory),
        }

    def test_valid_record_binds_scenario_and_all_evidence(self):
        scenario = scenario_for_seed(1, "production")
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            result_path = directory / "result.json"
            record = self._record(directory, scenario)

            _validate_result_record(result_path, record, scenario)

    def test_modified_evidence_is_rejected_on_resume(self):
        scenario = scenario_for_seed(1, "production")
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            result_path = directory / "result.json"
            record = self._record(directory, scenario)
            (directory / "physical_order_truth.log").write_text("edited")

            with self.assertRaisesRegex(ValueError, "evidence .*changed"):
                _validate_result_record(result_path, record, scenario)

    def test_result_from_another_scenario_is_rejected(self):
        scenario = scenario_for_seed(1, "production")
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            result_path = directory / "result.json"
            record = self._record(directory, scenario)

            with self.assertRaisesRegex(ValueError, "seed/profile"):
                _validate_result_record(
                    result_path, record, scenario_for_seed(2, "production")
                )

    def test_top_level_seed_or_profile_tampering_is_rejected(self):
        scenario = scenario_for_seed(1, "production")
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            result_path = directory / "result.json"
            record = self._record(directory, scenario)
            record["seed"] = 2

            with self.assertRaisesRegex(ValueError, "seed/profile"):
                _validate_result_record(result_path, record, scenario)



class CampaignAggregationTests(unittest.TestCase):
    def test_formal_rate_is_hidden_until_every_seed_is_terminal(self):
        summary = campaign_summary(
            (1, 2, 3),
            {1: {"status": "passed"}, 2: {"status": "failed"}},
            0.80,
        )

        self.assertIsNone(summary["formal_success_rate"])
        self.assertIsNone(summary["threshold_met"])
        self.assertEqual(summary["pending"], 1)

    def test_complete_small_campaign_is_not_reported_as_formal(self):
        summary = campaign_summary(
            (101,),
            {101: {"status": "passed"}},
            0.80,
        )

        self.assertFalse(summary["formal_eligible"])
        self.assertTrue(summary["all_configured_passed"])
        self.assertEqual(summary["completed_success_rate"], 1.0)
        self.assertIsNone(summary["formal_success_rate"])
        self.assertIsNone(summary["threshold_met"])

    def test_thirty_seed_contract_requires_twenty_four_passes(self):
        results = {
            seed: {"status": "passed" if seed <= 24 else "failed"}
            for seed in range(1, 31)
        }
        summary = campaign_summary(range(1, 31), results, 0.80)

        self.assertEqual(summary["required_successes"], 24)
        self.assertEqual(summary["formal_success_rate"], 0.80)
        self.assertTrue(summary["formal_eligible"])
        self.assertTrue(summary["threshold_met"])
        self.assertFalse(summary["unordered_coverage_met"])
        self.assertFalse(summary["qualification_met"])

    def test_formal_campaign_requires_spatial_coverage_as_well_as_success(self):
        results = {}
        for seed in range(1, 31):
            x = 0.60 + (seed % 5) * 0.07
            y = -0.35 + (seed % 6) * 0.14
            results[seed] = {
                "status": "passed",
                "physical_summary": {
                    "raw_targets": f"{x:.3f},{y:.3f},0.254"
                },
            }

        summary = campaign_summary(range(1, 31), results, 0.80)

        self.assertTrue(summary["threshold_met"])
        self.assertTrue(summary["unordered_coverage_met"])
        self.assertTrue(summary["qualification_met"])
        self.assertGreaterEqual(summary["raw_target_cells_50mm"], 12)
        self.assertGreaterEqual(summary["raw_target_x_span"], 0.10)
        self.assertGreaterEqual(summary["raw_target_y_span"], 0.30)
