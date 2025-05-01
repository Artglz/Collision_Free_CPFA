# rltrainer.py
# import numpy as np
# def train_step(actor_states, global_state):
#     print("\n[Python] train_step called!")
#     print(f"Actor states received: {len(actor_states)} robots")
#     for robot_id, state in actor_states.items():
#         print(f"Robot {robot_id}: {state}")
#     print(f"Global state received: {global_state}")
#     return True
import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
from collections import deque
import json
import time
from torch.utils.tensorboard import SummaryWriter


# Device configuration
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Paths for saving models and logs
MODEL_DIR = os.path.join(os.path.dirname(__file__), "models")
LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Hyperparameters
ACTOR_LR = 3e-4
CRITIC_LR = 3e-4
GAMMA = 0.99
GAE_LAMBDA = 0.95
PPO_EPSILON = 0.2
PPO_EPOCHS = 10
BATCH_SIZE = 64
ENTROPY_COEF = 0.01
VALUE_COEF = 0.5
MAX_GRAD_NORM = 0.5
MEMORY_SIZE = 10000  # Size of replay buffer

# Neural network architectures
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.mean = nn.Linear(64, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mean = self.mean(x)
        std = torch.exp(self.log_std.clamp(-20, 2))
        return mean, std
    
    def get_action(self, state, deterministic=False):
        mean, std = self(state)
        if deterministic:
            return mean
        else:
            dist = Normal(mean, std)
            action = dist.sample()
            action[..., 0] = torch.clamp(action[..., 0], -180.0, 180.0)
            action[..., 1] = torch.clamp(action[..., 1], 2.0, 32.0) 
        
            return action
        


        return action
    
    def evaluate(self, state, action):
        mean, std = self(state)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1, keepdim=True)
        entropy = dist.entropy().sum(dim=-1, keepdim=True)
        return log_prob, entropy

class Critic(nn.Module):
    def __init__(self, state_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.value = nn.Linear(64, 1)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        value = self.value(x)
        return value

# Memory buffer for storing experiences
class Memory:
    def __init__(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.next_states = []
        self.log_probs = []
        self.dones = []
        self.global_states = []
        # self.robot_ids = []
    
    def add(self, state, action, reward, next_state, log_prob, done, global_state, robot_id):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.next_states.append(next_state)
        self.log_probs.append(log_prob)
        self.dones.append(done)
        self.global_states.append(global_state)
        # self.robot_ids.append(robot_id)
    
    def clear(self):
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.next_states.clear()
        self.log_probs.clear()
        self.dones.clear()
        self.global_states.clear()
        # self.robot_ids.clear()
    
    def __len__(self):
        return len(self.states)

class RLTrainer:
    def __init__(self):
        # State dimensions
        self.actor_state_dim = 6  # distance_to_nest, timesteps_returning, collisions, path_efficiency, angular_deviation
        self.global_state_dim = 3  # nest_congestion_index, mean_path_efficiency, total_collisions
        self.action_dim = 2  # Steering angle and speed adjustments
        self.tb_writer = SummaryWriter(log_dir=LOG_DIR)

        # Initialize memory buffer
        self.memory = Memory()
        
        # Initialize actor and critic networks
        # self.actors = {}  # Maps robot_id to an actor network
        # self.actor_optimizers = {}
        # self.actor = Actor(self.actor_state_dim + self.global_state_dim, self.action_dim).to(device)
        self.actor = Actor(self.actor_state_dim, self.action_dim).to(device)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=ACTOR_LR)
        self.critic = Critic(self.actor_state_dim + self.global_state_dim).to(device)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=CRITIC_LR)
        
        # Tracking variables
        self.episode = 0
        self.step_count = 0
        self.rewards_history = deque(maxlen=100)
        self.collision_history = deque(maxlen=100)
        self.efficiency_history = deque(maxlen=100)
        
        # Load previous models if they exist
        self.load_models()
        
        # Logger
        self.log_file = os.path.join(LOG_DIR, f"training_log_{int(time.time())}.json")
        self.log_data = []
    
    # def get_actor(self, robot_id):
    #     """Get or create an actor for the specified robot"""
    #     if robot_id not in self.actors:
    #         self.actors[robot_id] = Actor(self.actor_state_dim + self.global_state_dim, self.action_dim).to(device)
    #         self.actor_optimizers[robot_id] = optim.Adam(self.actors[robot_id].parameters(), lr=ACTOR_LR)
    #     return self.actors[robot_id]
    def get_actor(self):
        return self.actor

    def calc_returns(self, rewards, dones, values):
        """Calculate returns with Generalized Advantage Estimation (GAE)"""
        returns = []
        advantages = []
        gae = 0
        
        for i in reversed(range(len(rewards))):
            if i == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[i + 1].item()
            
            delta = rewards[i] + GAMMA * next_value * (1 - dones[i]) - values[i].item()
            gae = delta + GAMMA * GAE_LAMBDA * (1 - dones[i]) * gae
            returns.insert(0, gae + values[i].item())
            advantages.insert(0, gae)
        
        return torch.tensor(returns, dtype=torch.float32).to(device), torch.tensor(advantages, dtype=torch.float32).to(device)
    
    def update_policy(self):
        """Update the policy using PPO algorithm"""
        if len(self.memory) < BATCH_SIZE:
            return  # Not enough samples for training
            
        # Convert memory data to tensors
        states = torch.FloatTensor(np.array(self.memory.states)).to(device)
        actions = torch.FloatTensor(np.array(self.memory.actions)).to(device)
        rewards = torch.FloatTensor(np.array(self.memory.rewards)).to(device)
        next_states = torch.FloatTensor(np.array(self.memory.next_states)).to(device)
        old_log_probs = torch.FloatTensor(np.array(self.memory.log_probs)).to(device)
        dones = torch.FloatTensor(np.array(self.memory.dones)).to(device)
        
        # Calculate values and returns
        values = self.critic(states).detach()
        returns, advantages = self.calc_returns(rewards, dones, values)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # # Group training data by robot_id
        # robot_data = {}
        # for i, robot_id in enumerate(robot_ids):
        #     if robot_id not in robot_data:
        #         robot_data[robot_id] = {'indices': []}
        #     robot_data[robot_id]['indices'].append(i)
        
        # Train for several epochs
        for _ in range(PPO_EPOCHS):
            # Update the centralized critic
            value_pred = self.critic(states)
            value_loss = F.mse_loss(value_pred, returns.unsqueeze(1))
            self.tb_writer.add_scalar('Loss/Critic', value_loss.item(), self.step_count)

            self.critic_optimizer.zero_grad()
            value_loss.backward()
            nn.utils.clip_grad_norm_(self.critic.parameters(), MAX_GRAD_NORM)
            self.critic_optimizer.step()

            # Update the shared actor (decentralized execution)
            new_log_probs, entropy = self.actor.evaluate(states, actions)
            ratio = torch.exp(new_log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1.0 - PPO_EPSILON, 1.0 + PPO_EPSILON) * advantages
            actor_loss = -torch.min(surr1, surr2).mean() - ENTROPY_COEF * entropy.mean()

            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), MAX_GRAD_NORM)
            self.actor_optimizer.step()

            self.tb_writer.add_scalar('Loss/Actor', actor_loss.item(), self.step_count)
        
        # Clear memory after update
        self.memory.clear()
        
        # Save models periodically
        if self.step_count % 1000 == 0:
            self.save_models()
            self.save_logs()
    
    def calculate_reward(self, prev_state, curr_state, global_state):
        """Calculate reward based on state transitions"""
        # Extract relevant metrics
        distance_change = prev_state[0] - curr_state[0]  # Reward for getting closer to nest
        collisions_occurred = curr_state[2] > prev_state[2]  # Penalty for collisions
        path_efficiency = curr_state[3]  # Reward for efficient path
        angular_deviation = curr_state[4]  # Penalty for deviating from optimal path
        nest_congestion = global_state[0]  # Penalty for approaching congested nest
        reached_nest = curr_state[5] # Check if robot reached the nest
        # Reward components
        reward_distance = 2.0 * distance_change  # Positive reward for approaching nest
        reward_collision = -5.0 * curr_state[2]  # Collision penalty
        reward_efficiency = 1.0 * path_efficiency  # Efficiency bonus
        reward_deviation = -0.75 * angular_deviation  # Deviation penalty
        reward_congestion = -1.0 * nest_congestion * max(0, 1.0 - curr_state[0])  # Congestion penalty (increases as robot gets closer to nest)
        reward_reach_nest = 20.0 if reached_nest >= 0.5 else 0.0
        
        # Total reward
        reward = reward_distance + reward_collision + reward_efficiency + reward_deviation + reward_congestion + reward_reach_nest
        
        return reward
    
    def get_action(self, robot_id, state, global_state, training=True):
        """Get action for a robot based on its current state"""
        # combined_state = np.concatenate([state, global_state])
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)
        
        # actor = self.get_actor(robot_id)
        actor = self.get_actor()
        with torch.no_grad():
            if training:
                action = actor.get_action(state_tensor, deterministic=False)
                log_prob, _ = actor.evaluate(state_tensor, action)
            else:
                action = actor.get_action(state_tensor, deterministic=True)
                log_prob = torch.zeros(1)

        return action.squeeze().cpu().numpy(), log_prob.item()
    
    def train_step(self, actor_states, global_state):
        """Process a training step from ARGoS simulation"""
        self.step_count += 1
        
        # Convert global state to numpy array
        global_state_array = np.array([
            global_state[0],  # nest_congestion_index
            global_state[1],  # mean_path_efficiency
            global_state[2]   # total_collisions
        ])
        
        # Process each robot's state and determine actions
        actions = {}
        new_states = {}
        
        # Keep track of metrics for logging
        episode_collisions = 0
        episode_efficiency = 0
        robot_count = 0
        
        # First, convert robot states and store prev_states
        for robot_id, state in actor_states.items():
            # Convert state to numpy array
            robot_state = np.array([
                state[0],  # distance_to_nest
                state[1],  # timesteps_returning
                state[2],  # collisions
                state[3],  # path_efficiency
                state[4],   # angular_deviation
                state[5]   # reached_nest
            ])
            
            # Store the state
            new_states[robot_id] = robot_state
            
            # Track metrics
            episode_collisions += state[2]
            episode_efficiency += state[3]
            robot_count += 1
            
            # Check if we have previous state for this robot
            if hasattr(self, 'prev_states') and robot_id in self.prev_states:
                prev_state = self.prev_states[robot_id]
                
                # Calculate reward
                reward = self.calculate_reward(prev_state, robot_state, global_state_array)
                
                # Get next action
                action, log_prob = self.get_action(robot_id, robot_state, global_state_array)
                actions[robot_id] = action
                
                # Determine if this is a terminal state (reached nest)
                done = robot_state[5] == 1  # Distance to nest is very small
                
                # Store experience in memory
                combined_state = np.concatenate([prev_state, global_state_array])
                combined_next_state = np.concatenate([robot_state, global_state_array])
                self.memory.add(combined_state, action, reward, combined_next_state, log_prob, done, global_state_array, robot_id)
                
                # Add reward to history for tracking
                self.rewards_history.append(reward)
            else:
                # First encounter with this robot, just get an action
                action, _ = self.get_action(robot_id, robot_state, global_state_array)
                actions[robot_id] = action
        
        # Store current states as previous states for next step
        if not hasattr(self, 'prev_states'):
            self.prev_states = {}
        self.prev_states = new_states
        
        # Update policy if enough steps have been accumulated
        if len(self.memory) >= BATCH_SIZE:
            self.update_policy()
        
        # Log metrics periodically
        if self.step_count % 100 == 0:
            if robot_count > 0:
                avg_efficiency = episode_efficiency / robot_count
                avg_collisions = episode_collisions / robot_count
                self.efficiency_history.append(avg_efficiency)
                self.collision_history.append(avg_collisions)
                
                self.log_data.append({
                    'step': self.step_count,
                    'avg_reward': np.mean(self.rewards_history) if self.rewards_history else 0,
                    'avg_efficiency': avg_efficiency,
                    'avg_collisions': avg_collisions,
                    'nest_congestion': global_state_array[0],
                    'mean_path_efficiency': global_state_array[1],
                    'total_collisions': global_state_array[2]
                })
                self.tb_writer.add_scalar('Reward/Avg', np.mean(self.rewards_history), self.step_count)
                self.tb_writer.add_scalar('Efficiency/Avg', avg_efficiency, self.step_count)
                self.tb_writer.add_scalar('Collisions/Avg', avg_collisions, self.step_count)
                self.tb_writer.add_scalar('Global/NestCongestion', global_state_array[0], self.step_count)
                self.tb_writer.add_scalar('Global/MeanPathEfficiency', global_state_array[1], self.step_count)
                self.tb_writer.add_scalar('Global/TotalCollisions', global_state_array[2], self.step_count)                
                # print(f"Step {self.step_count}, Avg Reward: {np.mean(self.rewards_history):.3f}, "
                #       f"Efficiency: {avg_efficiency:.3f}, Collisions: {avg_collisions:.2f}")
        
        # Encode actions as a JSON-serializable dictionary for C++
        action_dict = {robot_id: action.tolist() for robot_id, action in actions.items()}
        return action_dict
    
    def save_models(self):
        """Save all models to disk"""
        # Save actors
        # for robot_id, actor in self.actors.items():
        #     torch.save(actor.state_dict(), os.path.join(MODEL_DIR, f"actor_{robot_id}.pt"))
        torch.save(self.actor.state_dict(), os.path.join(MODEL_DIR, "actor_shared.pt"))
        # Save critic
        torch.save(self.critic.state_dict(), os.path.join(MODEL_DIR, "critic.pt"))
        self.episode += 1 # Increment episode count after simulation ends
        # Save training state
        training_state = {
            'episode': self.episode,
            'step_count': self.step_count,
        }
        with open(os.path.join(MODEL_DIR, "training_state.json"), 'w') as f:
            json.dump(training_state, f)

        self.tb_writer.close()

    
    def load_models(self):
        """Load models from disk if they exist"""
        # Load training state
        training_state_path = os.path.join(MODEL_DIR, "training_state.json")
        if os.path.exists(training_state_path):
            with open(training_state_path, 'r') as f:
                training_state = json.load(f)
                self.episode = training_state.get('episode', 0)
                self.step_count = training_state.get('step_count', 0)
                
                # Load actors for known robots
                # for robot_id in training_state.get('robot_ids', []):
                #     actor_path = os.path.join(MODEL_DIR, f"actor_{robot_id}.pt")
                #     if os.path.exists(actor_path):
                #         actor = Actor(self.actor_state_dim + self.global_state_dim, self.action_dim).to(device)
                #         actor.load_state_dict(torch.load(actor_path))
                #         self.actors[robot_id] = actor
                #         self.actor_optimizers[robot_id] = optim.Adam(actor.parameters(), lr=ACTOR_LR)
                actor_path = os.path.join(MODEL_DIR, "actor_shared.pt")
                if os.path.exists(actor_path):
                    self.actor.load_state_dict(torch.load(actor_path))

            # Load critic
            critic_path = os.path.join(MODEL_DIR, "critic.pt")
            if os.path.exists(critic_path):
                self.critic.load_state_dict(torch.load(critic_path))
    
    def save_logs(self):
        """Save training logs to disk"""
        with open(self.log_file, 'w') as f:
            json.dump(self.log_data, f)

# Global trainer instance
trainer = RLTrainer()

def save_models():
    """Interface function to save models from C++"""
    trainer.save_models()
    trainer.save_logs()

def train_step(actor_states, global_state):
    """
    Interface function called from C++
    
    Parameters:
    - actor_states: Dictionary mapping robot IDs to their states
    - global_state: List of global state variables
    
    Returns:
    - Dictionary mapping robot IDs to actions
    """
    return trainer.train_step(actor_states, global_state)

# Testing function for local development
def test():
    """Simple test to ensure the code runs"""
    # Sample actor states
    actor_states = {
        'robot_0': [1.5, 10, 0, 0.8, 0.2],  # distance, timesteps, collisions, efficiency, deviation
        'robot_1': [2.0, 5, 1, 0.6, 0.3]
    }
    
    # Sample global state
    global_state = [0.3, 0.7, 5]  # congestion, efficiency, collisions
    
    # Call train_step
    actions = train_step(actor_states, global_state)
    print(f"Generated actions: {actions}")
    
    # Test a few more steps
    for i in range(5):
        # Update states
        actor_states['robot_0'][0] -= 0.1  # Getting closer to nest
        actor_states['robot_1'][0] -= 0.15
        
        # Add a collision to robot_0
        if i == 2:
            actor_states['robot_0'][2] += 1
        
        actions = train_step(actor_states, global_state)
        print(f"Step {i+1}, Actions: {actions}")

# Only run test if executed directly
if __name__ == "__main__":
    test()