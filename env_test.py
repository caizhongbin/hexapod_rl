import gym
import torch as th
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from hexapod_env.hexapod_loc.envs.hexapod_gym_env import HexapodGymEnv
import os
# env = DummyVecEnv([lambda: HexapodGymEnv()])
env = HexapodGymEnv()
# env = VecNormalize(env, norm_obs=True, norm_reward=False,
#                    clip_obs=5.)

policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[dict(pi=[256, 128], vf=[256, 128])])
model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, learning_rate=2.5e-4, verbose=1, tensorboard_log="./hexapod_tensorboard_test/")
model.learn(
    total_timesteps=2048*30, tb_log_name="first_run"
)
log_dir = "/home/czbfy/hexapod_rl"
model.save(log_dir + "ppo_hexapod")
# stats_path = os.path.join(log_dir, "vec_normalize.pkl")
# env.save(stats_path)

del model, env
# env = DummyVecEnv([lambda: HexapodGymEnv(render=True)])
env = HexapodGymEnv(render=True)
# env = VecNormalize.load(stats_path, env)
# env.training = False
# env.norm_reward = False

model = PPO.load(log_dir + "ppo_hexapod", env=env)
obs = env.reset()
for i in range(10):
    action, _states = model.predict(obs)
    print(action)
    obs, rewards, dones, info = env.step(action)
    print(rewards)
    print(dones)
    env.render()
# stats_path = os.path.join(log_dir, "vec_normalize.pkl")
# env.save(stats_path)

# GO_LEFT=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
# n_steps = 20
# for step in range(n_steps):
#   print("Step {}".format(step + 1))
#   obs, reward, done, info = env.step(GO_LEFT)
#   print('obs=', obs, 'reward=', reward, 'done=', done)
#   env.render()
#   if done:
#     print("Goal reached!", "reward=", reward)
#     break

# obs = env.reset()
# return_obs, reward, done, info = env.step(GO_LEFT)
# print(return_obs.shape)
# print(return_obs)
# print(env.observation_space.contains(return_obs))
# env.render()
# print(env.observation_space)
# print(env.observation_space.sample())

# print(env.action_space)
# print(env.action_space.sample())
# It will check your custom environment and output additional warnings if needed
#check_env(env)