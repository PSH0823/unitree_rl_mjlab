"""Trainable dense GAT actor used by RSL-RL.

The graph is small and fixed-size, so a dense implementation avoids the
torch-geometric/scatter dependency and is substantially cheaper for thousands of
parallel environments.
"""

from __future__ import annotations

import torch
from rsl_rl.models import MLPModel
from rsl_rl.utils import unpad_trajectories


class DenseGATEncoder(torch.nn.Module):
  """psi1/psi2/psi3 graph attention encoder from the reference PENN model."""

  def __init__(self, embedding_dim: int = 16) -> None:
    super().__init__()
    # z_ij = node-label_i(3), node-label_j(3), edge_ij(5).
    self.psi1 = torch.nn.Sequential(
      torch.nn.Linear(11, 64), torch.nn.ReLU(), torch.nn.Linear(64, embedding_dim)
    )
    self.psi2 = torch.nn.Sequential(
      torch.nn.Linear(embedding_dim, 16),
      torch.nn.ReLU(),
      torch.nn.Linear(16, 1),
    )
    self.psi3 = torch.nn.Sequential(
      torch.nn.Linear(embedding_dim, 64),
      torch.nn.ReLU(),
      torch.nn.Linear(64, embedding_dim),
    )

  def forward(self, nodes: torch.Tensor) -> torch.Tensor:
    """Encode nodes shaped ``[..., N, 9]`` and return the robot-node embedding."""
    labels = nodes[..., :3]
    pos = nodes[..., 3:5]
    radius = nodes[..., 5]
    vel = nodes[..., 6:8]
    valid = nodes[..., 8] > 0.5
    n = nodes.shape[-2]

    li = labels.unsqueeze(-2).expand(*labels.shape[:-2], n, n, 3)
    lj = labels.unsqueeze(-3).expand(*labels.shape[:-2], n, n, 3)
    delta_pos = pos.unsqueeze(-3) - pos.unsqueeze(-2)  # j - i
    center_dist = torch.linalg.vector_norm(delta_pos, dim=-1, keepdim=True)
    surface_dist = center_dist - (
      radius.unsqueeze(-1) + radius.unsqueeze(-2)
    ).unsqueeze(-1)
    delta_vel = vel.unsqueeze(-3) - vel.unsqueeze(-2)
    z_ij = torch.cat((li, lj, delta_pos, surface_dist, delta_vel), dim=-1)

    messages = self.psi1(z_ij)
    logits = self.psi2(messages).squeeze(-1)
    edge_valid = valid.unsqueeze(-1) & valid.unsqueeze(-2)
    eye = torch.eye(n, dtype=torch.bool, device=nodes.device)
    edge_valid = edge_valid & ~eye
    logits = logits.masked_fill(~edge_valid, -1.0e9)
    weights = torch.softmax(logits, dim=-1)
    weights = weights * edge_valid
    weights = weights / weights.sum(dim=-1, keepdim=True).clamp_min(1.0e-8)
    updated = (weights.unsqueeze(-1) * self.psi3(messages)).sum(dim=-2)
    return updated[..., 0, :]


class GATActorModel(MLPModel):
  """RSL-RL actor that learns the GAT jointly with the policy head."""

  def __init__(
    self,
    *args,
    num_nodes: int = 12,
    node_dim: int = 9,
    gat_embedding_dim: int = 16,
    local_state_dim: int = 15,
    **kwargs,
  ) -> None:
    self.num_nodes = num_nodes
    self.node_dim = node_dim
    self.gat_embedding_dim = gat_embedding_dim
    self.local_state_dim = local_state_dim
    super().__init__(*args, **kwargs)
    self.gat = DenseGATEncoder(gat_embedding_dim)

  def _get_latent_dim(self) -> int:
    return self.gat_embedding_dim + self.local_state_dim

  def get_latent(self, obs, masks=None, hidden_state=None) -> torch.Tensor:
    obs = unpad_trajectories(obs, masks) if masks is not None else obs
    flat = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
    flat = self.obs_normalizer(flat)
    graph_dim = self.num_nodes * self.node_dim
    nodes = flat[..., :graph_dim].reshape(*flat.shape[:-1], self.num_nodes, self.node_dim)
    local = flat[..., graph_dim : graph_dim + self.local_state_dim]
    return torch.cat((self.gat(nodes), local), dim=-1)
