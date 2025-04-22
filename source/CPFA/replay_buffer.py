# replay_buffer.py
import random
import numpy as np

class ReplayBuffer:
    def __init__(self, max_size=100000):
        self.buffer = []
        self.max_size = max_size
        self.position = 0  # where to overwrite next

    def add(self, actor_states, global_state, action, reward, next_actor_states, next_global_state, done):
        """Save a transition tuple"""
        data = (actor_states, global_state, action, reward, next_actor_states, next_global_state, done)

        if len(self.buffer) < self.max_size:
            self.buffer.append(data)
        else:
            self.buffer[self.position] = data
            self.position = (self.position + 1) % self.max_size

    def sample(self, batch_size):
        """Randomly sample a batch"""
        batch = random.sample(self.buffer, batch_size)
        actor_states, global_states, actions, rewards, next_actor_states, next_global_states, dones = map(np.array, zip(*batch))
        return actor_states, global_states, actions, rewards, next_actor_states, next_global_states, dones

    def size(self):
        return len(self.buffer)

    def ready(self, batch_size):
        return self.size() >= batch_size
