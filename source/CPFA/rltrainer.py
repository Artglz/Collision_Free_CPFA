import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
from collections import deque
import pickle
import random
import threading
from torch.utils.tensorboard import SummaryWriter

# Constants
ACTOR_LR = 3e-4
CRITIC_LR = 3e-4
GAMMA = 0.99
LAMBDA = 0.95
EPSILON = 0.2  # PPO clipping parameter
ENTROPY_COEF = 0.01
VALUE_COEF = 0.5
MAX_BUFFER_SIZE = 10000
BATCH_SIZE = 64
UPDATE_EVERY = 100  # Update policy after collecting this many samples
CHECKPOINT_DIR = "source/CPFA/models"
LOG_EVERY = 10  # Log stats every N episodes

# Ensure checkpoint directory exists
os.makedirs(CHECKPOINT_DIR, exist_ok=True)

# Create log file
log_file = open(f"{CHECKPOINT_DIR}/training_log.txt", "a")

def log(message):
    """Write message to log file and print to console"""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    formatted_message = f"[{timestamp}] {message}"
    print(formatted_message)
    log_file.write(formatted_message + "\n")
    log_file.flush()

# Neural Network Models
class ActorNetwork(nn.Module):
    def __init__(self, input_dim=6, hidden_dim=128, action_dim=2):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        
        # Mean and log_std for continuous actions
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)
        
        # Initialize weights
        nn.init.orthogonal_(self.fc1.weight, gain=np.sqrt(2))
        nn.init.orthogonal_(self.fc2.weight, gain=np.sqrt(2))
        nn.init.orthogonal_(self.mean_head.weight, gain=0.01)
        nn.init.orthogonal_(self.log_std_head.weight, gain=0.01)
        
        # Action bounds (degree(-180 to 180) and speed(2 to 32))
        self.action_bounds = torch.tensor([[-180.0, 2.0], [180.0, 32.0]], dtype=torch.float32)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        
        # mean = self.mean_head(x)
        mean = torch.tanh(self.mean_head(x))  # <== constrain to [-1, 1]
        log_std = self.log_std_head(x)
        log_std = torch.clamp(log_std, -20, 2)  # Prevent too small/large std
        
        return mean, log_std
    
    def get_action(self, state, deterministic=False):
        mean, log_std = self.forward(state)
        
        if deterministic:
            # For deterministic action, just use the mean
            action = mean
        else:
            # Sample from Gaussian distribution
            std = log_std.exp()
            dist = Normal(mean, std)
            action = dist.sample()
            action = torch.clamp(action, -1.0, 1.0)
        
        # Scale actions to the proper range
        scaled_action = self.scale_action(action)
        
        return scaled_action, mean, log_std
    
    def evaluate_action(self, state, action):
        """Calculate log probability and entropy for given state-action pair"""
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        # Unscale action from environment range to network range
        action = self.unscale_action(action)
        
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy
    
    def scale_action(self, action):
        """Scale action from network output to environment range"""
        # Convert to numpy for easier handling
        if isinstance(action, torch.Tensor):
            action = action.detach().cpu().numpy()
            
        # Scale each dimension independently
        scaled_action = np.zeros_like(action)
        
        # Degree: Transform from [-1, 1] to [-180, 180]
        scaled_action[..., 0] = np.clip(action[..., 0], -1, 1) * 180.0
        
        # Speed: Transform from [-1, 1] to [2, 32]
        scaled_action[..., 1] = (np.clip(action[..., 1], -1, 1) + 1) / 2 * (32.0 - 2.0) + 2.0
        
        return scaled_action
    
    def unscale_action(self, action):
        """Unscale action from environment range to network range (approximately [-1, 1])"""
        # Convert to torch if numpy
        if isinstance(action, np.ndarray):
            action = torch.FloatTensor(action)
            
        unscaled = torch.zeros_like(action)
        
        # Degree: Transform from [-180, 180] to [-1, 1]
        unscaled[..., 0] = torch.clamp(action[..., 0] / 180.0, -1, 1)
        
        # Speed: Transform from [2, 32] to [-1, 1]
        unscaled[..., 1] = (torch.clamp(action[..., 1], 2, 32) - 2.0) / (32.0 - 2.0) * 2 - 1
        
        return unscaled

class CriticNetwork(nn.Module):
    def __init__(self, input_dim=3, hidden_dim=128):
        super(CriticNetwork, self).__init__()
        # Global state input (nest_congestion, mean_efficiency, total_collisions)
        self.global_fc1 = nn.Linear(input_dim, hidden_dim // 2)
        
        # Actor state input
        self.actor_fc1 = nn.Linear(6, hidden_dim // 2)
        
        # Combined layers
        self.combined_fc = nn.Linear(hidden_dim, hidden_dim)
        self.value_head = nn.Linear(hidden_dim, 1)
        
        # Initialize weights
        nn.init.orthogonal_(self.global_fc1.weight, gain=np.sqrt(2))
        nn.init.orthogonal_(self.actor_fc1.weight, gain=np.sqrt(2))
        nn.init.orthogonal_(self.combined_fc.weight, gain=np.sqrt(2))
        nn.init.orthogonal_(self.value_head.weight, gain=1.0)
        
    def forward(self, global_state, actor_state):
        # Process global state
        g = F.relu(self.global_fc1(global_state))
        
        # Process actor state
        a = F.relu(self.actor_fc1(actor_state))
        
        # Combine
        combined = torch.cat([g, a], dim=1)
        combined = F.relu(self.combined_fc(combined))
        
        # Output value
        value = self.value_head(combined)
        
        return value

class PPOAgent:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        log(f"Using device: {self.device}")
        print(f"Looking for models in: {CHECKPOINT_DIR}")

        # Initialize networks
        self.actor = ActorNetwork().to(self.device)
        self.critic = CriticNetwork().to(self.device)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=ACTOR_LR)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=CRITIC_LR)
        
        # Experience buffer
        self.reset_buffers()
        self.writer = SummaryWriter(log_dir=os.path.join(CHECKPOINT_DIR, "tensorboard"))

        # Training stats
        self.stats = {
            "steps": 0,
            "updates": 0,
            "mean_reward": 0,
            "rewards": [],
            "collision_rate": 0,
            "path_efficiency": 0,
            "nest_congestion": 0
        }
        
        # Load model if exists
        self.load_models()
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
    def reset_buffers(self):
        """Reset experience buffers"""
        self.states = []
        self.global_states = []
        self.actions = []
        self.rewards = []
        self.values = []
        self.log_probs = []
        self.dones = []
        self.timesteps = 0
        
    def store_experience(self, state, global_state, action, reward, value, log_prob, done):
        """Store experience in buffer"""
        with self.lock:
            self.states.append(state)
            self.global_states.append(global_state)
            self.actions.append(action)
            self.rewards.append(reward)
            self.values.append(value)
            self.log_probs.append(log_prob)
            self.dones.append(done)
            
            self.timesteps += 1
            self.stats["steps"] += 1
            self.stats["rewards"].append(reward)
            
            # Keep buffer size in check
            if len(self.states) > MAX_BUFFER_SIZE:
                self.states.pop(0)
                self.global_states.pop(0)
                self.actions.pop(0)
                self.rewards.pop(0)
                self.values.pop(0)
                self.log_probs.pop(0)
                self.dones.pop(0)
    
    def compute_returns(self, next_value, gamma=GAMMA, lambda_=LAMBDA):
        """Compute GAE (Generalized Advantage Estimation)"""
        returns = []
        gae = 0
        
        # Convert to numpy arrays for easier manipulation
        rewards = np.array(self.rewards)
        values = np.array(self.values + [next_value])
        dones = np.array(self.dones + [0])
        
        # Work backwards to compute returns and GAE
        for step in reversed(range(len(rewards))):
            delta = rewards[step] + gamma * values[step + 1] * (1 - dones[step]) - values[step]
            gae = delta + gamma * lambda_ * (1 - dones[step]) * gae
            returns.insert(0, gae + values[step])
            
        return returns
    
    def update_policy(self, next_value=0):
        """Update actor and critic networks using PPO"""
        if len(self.states) < BATCH_SIZE:
            return  # Not enough samples yet
        
        with self.lock:
            # Compute returns
            returns = self.compute_returns(next_value)
            
            # Convert lists to tensors
            states = torch.FloatTensor(np.array(self.states)).to(self.device)
            global_states = torch.FloatTensor(np.array(self.global_states)).to(self.device)
            actions = torch.FloatTensor(np.array(self.actions)).to(self.device)
            old_log_probs = torch.FloatTensor(np.array(self.log_probs)).to(self.device)
            returns = torch.FloatTensor(returns).to(self.device)
            
            # Update policy for multiple epochs
            for _ in range(5):  # Typically 3-10 epochs
                # Generate random indices for batches
                indices = np.random.permutation(len(states))
                
                # Process in batches
                for start_idx in range(0, len(states), BATCH_SIZE):
                    idx = indices[start_idx:start_idx + BATCH_SIZE]
                    
                    batch_states = states[idx]
                    batch_global_states = global_states[idx]
                    batch_actions = actions[idx]
                    batch_old_log_probs = old_log_probs[idx]
                    batch_returns = returns[idx]
                    
                    # Get current policy evaluation
                    current_log_probs, entropy = self.actor.evaluate_action(batch_states, batch_actions)
                    current_values = self.critic(batch_global_states, batch_states).squeeze(-1)
                    
                    # Calculate advantage
                    advantages = batch_returns - current_values.detach()
                    
                    # Normalize advantages
                    if len(advantages) > 1:
                        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
                    
                    # PPO ratio and surrogate loss
                    ratio = torch.exp(current_log_probs - batch_old_log_probs)
                    surr1 = ratio * advantages
                    surr2 = torch.clamp(ratio, 1.0 - EPSILON, 1.0 + EPSILON) * advantages
                    
                    # Calculate losses
                    actor_loss = -torch.min(surr1, surr2).mean()
                    critic_loss = F.mse_loss(current_values, batch_returns)
                    entropy_loss = -entropy.mean()
                    
                    # Total loss
                    loss = actor_loss + VALUE_COEF * critic_loss + ENTROPY_COEF * entropy_loss
                    
                    # Update networks
                    self.actor_optimizer.zero_grad()
                    self.critic_optimizer.zero_grad()
                    loss.backward()
                    
                    # Clip gradients
                    torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 0.5)
                    torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
                    
                    self.actor_optimizer.step()
                    self.critic_optimizer.step()
            
            # Update training stats
            self.stats["updates"] += 1
            if len(self.stats["rewards"]) > 0:
                self.stats["mean_reward"] = np.mean(self.stats["rewards"][-100:])
            
            # Log stats periodically
            if self.stats["updates"] % LOG_EVERY == 0:
                log(f"Update {self.stats['updates']}, Steps: {self.stats['steps']}, "
                    f"Mean reward: {self.stats['mean_reward']:.4f}")
                self.writer.add_scalar("Reward/Mean", self.stats["mean_reward"], self.stats["updates"])
                self.writer.add_scalar("Loss/Actor", actor_loss.item(), self.stats["updates"])
                self.writer.add_scalar("Loss/Critic", critic_loss.item(), self.stats["updates"])
                self.writer.add_scalar("Loss/Entropy", entropy_loss.item(), self.stats["updates"])

            # Reset buffers after update
            self.reset_buffers()
            
            # Save models periodically
            if self.stats["updates"] % 10 == 0:
                self.save_models()
                
    def select_action(self, state, global_state, deterministic=False):
        """Select an action based on current policy"""
        with torch.no_grad():
            # Convert state to tensor
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            global_state_tensor = torch.FloatTensor(global_state).unsqueeze(0).to(self.device)
            
            # Get action from actor network
            action, mean, log_std = self.actor.get_action(state_tensor, deterministic)
            
            # Get value from critic network
            value = self.critic(global_state_tensor, state_tensor).item()
            
            # Calculate log probability of the action
            std = log_std.exp()
            dist = Normal(mean, std)
            unscaled_action = self.actor.unscale_action(torch.tensor(action))
            log_prob = dist.log_prob(unscaled_action.to(self.device)).sum(-1).item()
            
        return action.squeeze(0), value, log_prob
        
    def save_models(self):
        """Save model weights to disk"""
        try:
            torch.save(self.actor.state_dict(), f"{CHECKPOINT_DIR}/actor_model.pth")
            torch.save(self.critic.state_dict(), f"{CHECKPOINT_DIR}/critic_model.pth")
            with open(f"{CHECKPOINT_DIR}/stats.pkl", 'wb') as f:
                pickle.dump(self.stats, f)
            log(f"Models saved to {CHECKPOINT_DIR}")
        except Exception as e:
            log(f"Error saving models: {e}")
            
    def load_models(self):
        """Load model weights from disk if they exist"""
        try:
            if os.path.exists(f"{CHECKPOINT_DIR}/actor_model.pth"):
                self.actor.load_state_dict(torch.load(f"{CHECKPOINT_DIR}/actor_model.pth", 
                                                    map_location=self.device))
                log("Actor model loaded from disk")
                
            if os.path.exists(f"{CHECKPOINT_DIR}/critic_model.pth"):
                self.critic.load_state_dict(torch.load(f"{CHECKPOINT_DIR}/critic_model.pth", 
                                                     map_location=self.device))
                log("Critic model loaded from disk")
                
            if os.path.exists(f"{CHECKPOINT_DIR}/stats.pkl"):
                with open(f"{CHECKPOINT_DIR}/stats.pkl", 'rb') as f:
                    self.stats = pickle.load(f)
                log(f"Training stats loaded: Steps={self.stats['steps']}, Updates={self.stats['updates']}")
        except Exception as e:
            log(f"Error loading models: {e}")

# Create global agent
agent = PPOAgent()

# Experience memory for all robots
robot_memories = {}

# ==== C++ Interface Functions ====

def get_actions(actor_states, global_state):
    """
    Interface for C++ to get actions for all robots
    
    Args:
        actor_states: Dict of robot_id -> [distance_to_nest, timesteps_returning, 
                                          collisions, path_efficiency, angular_deviation, reached_nest]
        global_state: List [nest_congestion_index, mean_path_efficiency, total_collisions]
    
    Returns:
        Dict of robot_id -> [degree, speed]
    """
    actions = {}

    # Process each robot
    for robot_id, state in actor_states.items():
        # Check if we have previous state for this robot
        prev_state = None
        if robot_id in robot_memories:
            prev_state = robot_memories[robot_id].get('state', None)
        
        # Compute reward if we have previous state
        reward = 0
        if prev_state is not None:
            reward = compute_reward(prev_state, state, robot_memories[robot_id].get('global_state', global_state))
            
            # Store experience
            if 'action' in robot_memories[robot_id]:
                agent.store_experience(
                    prev_state,
                    robot_memories[robot_id].get('global_state', global_state),
                    robot_memories[robot_id]['action'],
                    reward,
                    robot_memories[robot_id].get('value', 0),
                    robot_memories[robot_id].get('log_prob', 0),
                    state[5] >= 0.5  # done if reached nest
                )
        
        # Get action for current state
        action, value, log_prob = agent.select_action(state, global_state)
        
        # Store memory for next time
        robot_memories[robot_id] = {
            'state': state,
            'global_state': global_state,
            'action': action,
            'value': value,
            'log_prob': log_prob
        }
        
        # Add action to return dict
        actions[robot_id] = action.tolist()
    
    # Periodically update policy
    if agent.timesteps >= UPDATE_EVERY:
        agent.update_policy()
    
    return actions

def compute_reward(prev_state, curr_state, global_state):
    """
    Compute reward for a transition
    
    Args:
        prev_state: Previous robot state
        curr_state: Current robot state
        global_state: Global state info
    
    Returns:
        float: Reward value
    """
    # Extract relevant metrics
    distance_change = prev_state[0] - curr_state[0]  # Reward for getting closer to nest
    collisions_occurred = curr_state[2] > prev_state[2]  # Penalty for collisions
    path_efficiency = curr_state[3]  # Reward for efficient path
    angular_deviation = curr_state[4]  # Penalty for deviating from optimal path
    nest_congestion = global_state[0]  # Penalty for approaching congested nest
    reached_nest = curr_state[5]  # Check if robot reached the nest
    
    # Reward components
    # reward_distance = 2.0 * distance_change  # Positive reward for approaching nest
    # reward_collision = -5.0 if collisions_occurred else 0.0  # Collision penalty
    # reward_efficiency = 1.0 * path_efficiency  # Efficiency bonus
    # reward_deviation = -0.75 * angular_deviation  # Deviation penalty
    # reward_congestion = -1.0 * nest_congestion * max(0, 1.0 - curr_state[0])
    reward_reach_nest = 20.0 if reached_nest >= 0.5 else 0.0

    reward_collision  = -1.0 if collisions_occurred else 0.0
    reward_deviation  = -0.25 * angular_deviation
    reward_congestion = -0.25 * nest_congestion * max(0, 1.0 - curr_state[0])
    reward_efficiency = 2.0 * path_efficiency
    reward_distance = 3.0 * distance_change

    
    # Total reward
    reward = (reward_distance + 
              reward_collision + 
              reward_efficiency + 
              reward_deviation + 
              reward_congestion + 
              reward_reach_nest)
    
    return reward

def save_models():
    """Interface for C++ to save models at the end of an experiment"""
    agent.save_models()
    if hasattr(self, 'writer'):
        self.writer.flush()
        self.writer.close()
    return True

# Initialize if running directly
if __name__ == "__main__":
    log("RL trainer initialized")