import unittest

from factory_agent.operator_state import OperatorState


class OperatorStateTests(unittest.TestCase):
    def test_completed_order_remains_queryable_after_active_id_clears(self):
        state = OperatorState()
        state.start_order("agent-order-1")
        state.finish_order(success=True, message="完成")
        state.active_order_id = ""

        self.assertEqual(state.tracked_order_id, "agent-order-1")
        self.assertIn("订单=agent-order-1", state.task_summary())
        self.assertIn("阶段=complete", state.task_summary())

    def test_new_order_replaces_previous_tracked_order(self):
        state = OperatorState()
        state.start_order("agent-order-1")
        state.finish_order(success=False, message="失败")
        state.start_order("agent-order-2")

        self.assertEqual(state.tracked_order_id, "agent-order-2")
        self.assertEqual(state.phase, "accepted")
        self.assertIn("确定性执行器接受", state.detail)


if __name__ == "__main__":
    unittest.main()
