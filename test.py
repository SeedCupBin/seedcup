from env import Env
from team_algorithm import MyCustomAlgorithm
import posix

def main(algorithm):
    seed = int.from_bytes(posix.urandom(4));
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

        print(f"Test #{i} completed. steps:", env.step_num, "Distance:", env.get_dis(), "Score:", score)

    final_score /= num_episodes
    avg_distance = total_distance / num_episodes
    avg_steps = total_steps / num_episodes

    print("\033[92mTest completed.\n\tSeed: {}\n\tTotal steps: {}\n\tFinal distance: {}\n\tFinal score: {}\033[0m".format(seed, avg_steps, avg_distance, final_score))

    env.close()
    # After exiting the loop, get the total steps and final distance

if __name__ == "__main__":
    # algorithm = PPOAlgorithm()
    algorithm = MyCustomAlgorithm()
    main(algorithm)