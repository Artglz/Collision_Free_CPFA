# rltrainer.py
# def train_step(actor_states, global_state):
#     print("\n[Python] train_step called!")
#     print(f"Actor states received: {len(actor_states)} robots")
#     for robot_id, state in actor_states.items():
#         print(f"Robot {robot_id}: {state}")
#     print(f"Global state received: {global_state}")
#     return True
from replay_buffer import ReplayBuffer

buffer = ReplayBuffer(max_size=100000)
BATCH_SIZE = 64
def train_step(actor_states, global_state):
    # Turn incoming states into numpy arrays
    obs = np.array(list(actor_states.values()), dtype=np.float32).flatten()
    gobs = np.array(global_state, dtype=np.float32)

    # Right now we don't yet have action, reward, next_obs (need to hook that up later)
    dummy_action = np.zeros(2)  # assuming 2D action space for now
    dummy_reward = 0.0
    dummy_done = False

    buffer.add(obs, gobs, dummy_action, dummy_reward, obs, gobs, dummy_done)

    # Check if we have enough to start training
    if buffer.ready(BATCH_SIZE):
        batch = buffer.sample(BATCH_SIZE)
        print(f"[Trainer] Sampled batch of {BATCH_SIZE}")
        # You would now pass this batch to your neural network to train
    else:
        print(f"[Trainer] Buffer size: {buffer.size()}/{BATCH_SIZE}")

    return True
