"""Small checks for the geometry and information boundary used by AIRCOORD."""
import unittest
import numpy as np
from aircoord import Config, TrafficEnv, compensate_report, goal_policy, swept_distances


class CoreTests(unittest.TestCase):
    def test_crossing_is_detected_between_endpoints(self):
        before = np.array([[-1., 0.], [1., 0.]])
        after = -before
        _, _, distance = swept_distances(before, after)
        self.assertAlmostEqual(float(distance[0]), 0.)

    def test_replay_recovers_state_without_disturbance(self):
        config = Config(delay_s=0.5)
        env = TrafficEnv(config)
        obs = env.reset(7)
        commands = []
        for _ in range(15):
            command = goal_policy(obs, config)
            commands.append(command.copy())
            obs, _, _ = env.step(command)
        stale = env.controller_observation('stale')
        np.testing.assert_array_equal(stale['own'], env.history[10])
        recovered = compensate_report(stale, commands, config)
        np.testing.assert_allclose(recovered['own'], env.state, atol=1e-12)
        self.assertAlmostEqual(recovered['observation_age_s'], 0.5)

    def test_unobserved_noise_is_not_replayed(self):
        config = Config(delay_s=0.5, accel_noise_std=0.3)
        env = TrafficEnv(config)
        obs = env.reset(7)
        commands = []
        for _ in range(15):
            command = goal_policy(obs, config)
            commands.append(command.copy())
            obs, _, _ = env.step(command)
        estimate = compensate_report(env.controller_observation('stale'), commands, config)
        self.assertGreater(np.max(np.abs(estimate['own'] - env.state)), 1e-6)

    def test_current_reference_has_zero_measurement_age(self):
        env = TrafficEnv(Config(delay_s=2.0))
        obs = env.reset(7)
        for _ in range(5):
            obs, _, _ = env.step(goal_policy(obs, env.c))
        self.assertEqual(env.controller_observation('current')['observation_age_s'], 0.)


if __name__ == '__main__':
    unittest.main()
