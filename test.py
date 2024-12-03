from env import Env
from config import Config
import sys
from team_algorithm import MyCustomAlgorithm

def main(algorithm):
    env = Env("env.cfg")
    cfg = Config("test.cfg")
    done = False
    num_episodes = cfg.GetAs('Rounds', int)
    epInfo = cfg.GetAs('InfoOutput.Episode', bool)
    result = cfg.GetAs('InfoOutput.Result', bool)
    notify = cfg.GetAs('Algorithm.Notify', bool)
    final_score = 0
    total_steps = 0
    total_distance = 0
    total_success = 0
    print("Seed = {}".format(env.seed))

    if notify: algorithm.NotifyTestBegin()

    for i in range(num_episodes):
        score = 0
        done = False
        first = True

        env.reset_episode()
        while not done:
            observation = env.get_observation()
            if notify and first: algorithm.NotifyRoundBegin(observation)
            first = False;
            action = algorithm.get_action(observation)
            obs = env.step(action)
            score += env.success_reward

            # Check if the episode has ended
            done = env.terminated

        total_steps += env.step_num
        total_distance += env.get_dis()
        final_score += score
        if env.step_num < env.max_steps:
            total_success += 1

        if notify: algorithm.NotifyRoundEnd([i, env.step_num, env.get_dis(), score]);
        if epInfo: print("\033[92mTest #{} completed.\n\tSteps used:\t{}\n\tDistance:\t{}\n\tEp. Score:\t{}\033[0m".format(i, env.step_num, env.get_dis(), score), flush=True)
        else: print("Round {}/{}: {:.2f}%".format(i, num_episodes, i / num_episodes * 100), file = sys.stderr, end = "\r")

    final_score /= num_episodes
    avg_distance = total_distance / num_episodes
    avg_steps = total_steps / num_episodes
    sxrate = total_success / num_episodes


    if notify: algorithm.NotifyTestEnd();
    if result: print("\033[92mTest completed.\n\tTest seed:\t{}\n\tAvg. steps:\t{}\t{}\n\tAvg. distance:\n\t Suc. Rate:\t{}\n\tFinal score:\t{}\033[0m".format(env.seed, avg_steps, avg_distance, sxrage, final_score))

    env.close()
    # After exiting the loop, get the total steps and final distance

if __name__ == "__main__":
    # algorithm = PPOAlgorithm()
    algorithm = MyCustomAlgorithm()
    main(algorithm)