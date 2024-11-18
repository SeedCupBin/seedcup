from env import Env
from team_algorithm import MyCustomAlgorithm
import sys

def main(algorithm):
    if sys.platform != 'win32':
        import posix;
        seed = int.from_bytes(posix.urandom(4));
    else:
        seed = 114514;
    env = Env(is_senior = False, seed = seed, gui = True)
    done = False
    num_episodes = 100
    final_score = 0
    total_steps = 0
    total_distance = 0
    print("Seed = {}".format(seed))

    for i in range(num_episodes):
        score = 0
        done = False
        first = True

        while not done:
            observation = env.get_observation()
            if first:
                algorithm.RoundNotify(observation)
                first = False;
            action = algorithm.get_action(observation)
            obs = env.step(action)
            score += env.success_reward

            # Check if the episode has ended
            done = env.terminated

        total_steps += env.step_num
        total_distance += env.get_dis()
        final_score += score

        print("\033[92mTest #{} completed.\n\tSteps used:\t{}\n\tDistance:\t{}\n\tEp. Score:\t{}\033[0m".format(i, env.step_num, env.get_dis(), score))

    final_score /= num_episodes
    avg_distance = total_distance / num_episodes
    avg_steps = total_steps / num_episodes

    print("\033[92mTest completed.\n\tTest seed:\t{}\n\tAvg. steps:\t{}\n\tAvg. distance:\t{}\n\tFinal score:\t{}\033[0m".format(seed, avg_steps, avg_distance, final_score))

    env.close()
    # After exiting the loop, get the total steps and final distance

if __name__ == "__main__":
    # algorithm = PPOAlgorithm()
    algorithm = MyCustomAlgorithm()
    main(algorithm)