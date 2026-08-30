import unittest

from factory_core.raw_part_perception import (
    PerceivedPart,
    PerceivedSelection,
    StableCandidateCollector,
    fresh_candidate_observations,
    merge_perceived_candidates,
    nearest_candidate,
    stable_candidates,
    stable_nearest_candidate,
)


class RawPartPerceptionTests(unittest.TestCase):
    def test_selection_preserves_fused_and_stable_candidate_sets(self):
        target = PerceivedPart(0.8, 0.0, 0.25)
        other = PerceivedPart(0.7, 0.2, 0.25)
        selection = PerceivedSelection(
            target=target,
            candidates=(target, other),
            stable_candidates=(target,),
        )

        self.assertEqual(selection.candidates, (target, other))
        self.assertEqual(selection.stable_candidates, (target,))

    def test_candidate_collector_waits_for_new_fused_frames(self):
        target = PerceivedPart(0.8, 0.0, 0.25)
        selection = PerceivedSelection(
            target=target,
            candidates=(target,),
            stable_candidates=(target,),
        )
        collector = StableCandidateCollector(required_updates=2)

        self.assertIsNone(collector.observe(selection, sequence=10))
        self.assertIsNone(collector.observe(selection, sequence=11))
        self.assertEqual(
            collector.observe(selection, sequence=12), selection
        )

    def test_candidate_collector_keeps_richest_multiview_selection(self):
        near = PerceivedPart(0.80, 0.00, 0.25)
        far = PerceivedPart(0.89, -0.18, 0.25)
        first = PerceivedSelection(
            target=near,
            candidates=(near,),
            stable_candidates=(near,),
        )
        richer = PerceivedSelection(
            target=near,
            candidates=(near, far),
            stable_candidates=(near, far),
        )
        latest_partial = PerceivedSelection(
            target=near,
            candidates=(near,),
            stable_candidates=(near,),
        )
        collector = StableCandidateCollector(required_updates=2)

        self.assertIsNone(collector.observe(first, sequence=20))
        self.assertIsNone(collector.observe(richer, sequence=21))
        self.assertEqual(
            collector.observe(latest_partial, sequence=22), richer
        )

    def test_candidate_collector_can_preserve_immediate_policy(self):
        target = PerceivedPart(0.8, 0.0, 0.25)
        selection = PerceivedSelection(
            target=target,
            candidates=(target,),
            stable_candidates=(target,),
        )
        collector = StableCandidateCollector(required_updates=0)

        self.assertEqual(collector.observe(selection, sequence=1), selection)

    def test_candidate_collector_rejects_invalid_inputs(self):
        target = PerceivedPart(0.8, 0.0, 0.25)
        empty = PerceivedSelection(
            target=target,
            candidates=(target,),
            stable_candidates=(),
        )
        with self.assertRaisesRegex(ValueError, "negative"):
            StableCandidateCollector(required_updates=-1)
        collector = StableCandidateCollector(required_updates=1)
        with self.assertRaisesRegex(ValueError, "stable"):
            collector.observe(empty, sequence=1)
        with self.assertRaisesRegex(ValueError, "negative"):
            collector.observe(
                PerceivedSelection(target, (target,), (target,)),
                sequence=-1,
            )

    def test_selects_nearest_reachable_candidate(self):
        selected = nearest_candidate(
            (
                PerceivedPart(0.79, 0.24, 0.25),
                PerceivedPart(0.80, -0.01, 0.25),
            ),
            (0.782, 0.0, 0.22),
            maximum_horizontal_distance=0.10,
        )
        self.assertEqual(selected, PerceivedPart(0.80, -0.01, 0.25))

    def test_multiview_fusion_merges_duplicates_and_keeps_occluded_parts(
        self,
    ):
        primary_only = PerceivedPart(0.74, -0.18, 0.25)
        shared_primary = PerceivedPart(0.80, 0.02, 0.251)
        shared_auxiliary = PerceivedPart(0.802, 0.018, 0.249)
        auxiliary_only = PerceivedPart(0.88, 0.21, 0.25)

        fused = merge_perceived_candidates(
            (
                (primary_only, shared_primary),
                (shared_auxiliary, auxiliary_only),
            ),
            maximum_distance=0.04,
        )

        self.assertEqual(len(fused), 3)
        self.assertIn(primary_only, fused)
        self.assertIn(auxiliary_only, fused)
        self.assertIn(PerceivedPart(0.801, 0.019, 0.250), fused)

    def test_multiview_fusion_does_not_create_parts_from_empty_views(self):
        visible = PerceivedPart(0.80, 0.0, 0.25)

        fused = merge_perceived_candidates(
            ((), (visible,), ()), maximum_distance=0.04
        )

        self.assertEqual(fused, (visible,))

    def test_multiview_fusion_rejects_invalid_association_distance(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            merge_perceived_candidates((), maximum_distance=0.0)

    def test_rejects_candidate_outside_partition(self):
        selected = nearest_candidate(
            (PerceivedPart(0.80, 0.25, 0.25),),
            (0.782, 0.0, 0.22),
            maximum_horizontal_distance=0.10,
        )
        self.assertIsNone(selected)

    def test_workspace_radius_accepts_a_candidate_without_a_taught_slot(self):
        selected = nearest_candidate(
            (
                PerceivedPart(0.62, 0.28, 0.25),
                PerceivedPart(0.90, -0.31, 0.25),
            ),
            (0.782, 0.0, 0.22),
            maximum_horizontal_distance=0.42,
        )
        self.assertEqual(selected, PerceivedPart(0.62, 0.28, 0.25))

    def test_rejects_invalid_distance(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            nearest_candidate(
                (),
                (0.0, 0.0, 0.0),
                maximum_horizontal_distance=0.0,
            )

    def test_stable_candidate_uses_temporal_median(self):
        selected = stable_nearest_candidate(
            (
                (PerceivedPart(0.801, 0.002, 0.251),),
                (PerceivedPart(0.798, -0.001, 0.249),),
                (PerceivedPart(0.800, 0.001, 0.250),),
            ),
            (0.80, 0.0, 0.25),
            maximum_horizontal_distance=0.10,
            required_observations=3,
            maximum_jitter=0.01,
        )
        self.assertEqual(selected, PerceivedPart(0.800, 0.001, 0.250))

    def test_returns_all_stable_candidates_in_policy_order(self):
        near = PerceivedPart(0.80, 0.01, 0.25)
        far = PerceivedPart(0.92, -0.08, 0.25)
        observations = (
            (
                PerceivedPart(0.801, 0.011, 0.250),
                PerceivedPart(0.921, -0.079, 0.250),
            ),
            (
                PerceivedPart(0.799, 0.009, 0.251),
                PerceivedPart(0.919, -0.081, 0.249),
            ),
            (near, far),
        )

        selected = stable_candidates(
            observations,
            (0.80, 0.0, 0.25),
            maximum_horizontal_distance=0.30,
            required_observations=3,
            maximum_jitter=0.01,
        )

        self.assertEqual(selected, (near, far))

    def test_rejects_a_jumping_candidate(self):
        selected = stable_nearest_candidate(
            (
                (PerceivedPart(0.80, 0.00, 0.25),),
                (PerceivedPart(0.80, 0.04, 0.25),),
                (PerceivedPart(0.80, 0.00, 0.25),),
            ),
            (0.80, 0.0, 0.25),
            maximum_horizontal_distance=0.10,
            required_observations=3,
            maximum_jitter=0.01,
        )
        self.assertIsNone(selected)

    def test_target_track_tolerates_one_missing_detection(self):
        selected = stable_nearest_candidate(
            (
                (
                    PerceivedPart(0.801, 0.002, 0.251),
                    PerceivedPart(0.80, 0.09, 0.25),
                ),
                (PerceivedPart(0.80, 0.09, 0.25),),
                (
                    PerceivedPart(0.798, -0.001, 0.249),
                    PerceivedPart(0.80, 0.09, 0.25),
                ),
                (
                    PerceivedPart(0.800, 0.001, 0.250),
                    PerceivedPart(0.80, 0.09, 0.25),
                ),
            ),
            (0.80, 0.0, 0.25),
            maximum_horizontal_distance=0.10,
            required_observations=3,
            maximum_jitter=0.01,
        )
        self.assertEqual(selected, PerceivedPart(0.800, 0.001, 0.250))

    def test_target_must_be_visible_in_latest_observation(self):
        selected = stable_nearest_candidate(
            (
                (PerceivedPart(0.800, 0.000, 0.250),),
                (PerceivedPart(0.801, 0.001, 0.250),),
                (PerceivedPart(0.799, -0.001, 0.250),),
                (PerceivedPart(0.80, 0.09, 0.25),),
            ),
            (0.80, 0.0, 0.25),
            maximum_horizontal_distance=0.10,
            required_observations=3,
            maximum_jitter=0.01,
        )
        self.assertIsNone(selected)

    def test_slow_camera_keeps_consensus_history_with_a_fresh_latest_frame(
        self
    ):
        first = (PerceivedPart(0.800, 0.000, 0.250),)
        second = (PerceivedPart(0.801, 0.001, 0.250),)
        latest = (PerceivedPart(0.799, -0.001, 0.250),)

        observations = fresh_candidate_observations(
            (
                (10.0, first),
                (11.5, second),
                (13.0, latest),
            ),
            requested_at=9.0,
            now=13.5,
            maximum_age=1.0,
            history_window=4.0,
        )

        self.assertEqual(observations, (first, second, latest))

    def test_slow_camera_rejects_a_stale_latest_frame(self):
        observations = fresh_candidate_observations(
            (
                (10.0, (PerceivedPart(0.800, 0.000, 0.250),)),
                (11.5, (PerceivedPart(0.801, 0.001, 0.250),)),
                (13.0, (PerceivedPart(0.799, -0.001, 0.250),)),
            ),
            requested_at=9.0,
            now=14.1,
            maximum_age=1.0,
            history_window=5.0,
        )

        self.assertEqual(observations, ())

    def test_history_window_cannot_weaken_freshness_contract(self):
        with self.assertRaisesRegex(ValueError, "at least"):
            fresh_candidate_observations(
                (),
                requested_at=0.0,
                now=1.0,
                maximum_age=2.0,
                history_window=1.0,
            )


if __name__ == "__main__":
    unittest.main()
