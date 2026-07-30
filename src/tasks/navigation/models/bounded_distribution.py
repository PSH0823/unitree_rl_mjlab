"""Tanh-squashed Gaussian distribution for bounded navigation actions."""

from __future__ import annotations

import torch
from rsl_rl.modules.distribution import Distribution
from torch import nn
from torch.distributions import Normal


class _TanhOutput(nn.Module):
  def forward(self, value: torch.Tensor) -> torch.Tensor:
    return torch.tanh(value)


class TanhGaussianDistribution(Distribution):
  """Diagonal Gaussian transformed to the exact ``[-1, 1]`` action domain."""

  def __init__(self, output_dim: int, init_std: float = 0.3) -> None:
    super().__init__(output_dim)
    self.log_std_param = nn.Parameter(
      torch.full((output_dim,), float(torch.log(torch.tensor(init_std))))
    )
    self._distribution: Normal | None = None

  @property
  def input_dim(self) -> int:
    return self.output_dim

  def update(self, mlp_output: torch.Tensor) -> None:
    std = torch.exp(self.log_std_param).expand_as(mlp_output)
    self._distribution = Normal(mlp_output, std)

  def sample(self) -> torch.Tensor:
    assert self._distribution is not None
    return torch.tanh(self._distribution.sample())

  def deterministic_output(self, mlp_output: torch.Tensor) -> torch.Tensor:
    return torch.tanh(mlp_output)

  def as_deterministic_output_module(self) -> nn.Module:
    return _TanhOutput()

  @property
  def mean(self) -> torch.Tensor:
    assert self._distribution is not None
    return torch.tanh(self._distribution.mean)

  @property
  def std(self) -> torch.Tensor:
    assert self._distribution is not None
    return self._distribution.stddev

  @property
  def entropy(self) -> torch.Tensor:
    # A transformed Normal has no analytic entropy. The base entropy is a
    # stable exploration surrogate and is sufficient for PPO regularization.
    assert self._distribution is not None
    return self._distribution.entropy().sum(dim=-1)

  @property
  def params(self) -> tuple[torch.Tensor, ...]:
    assert self._distribution is not None
    return self._distribution.mean, self._distribution.stddev

  def log_prob(self, outputs: torch.Tensor) -> torch.Tensor:
    assert self._distribution is not None
    bounded = outputs.clamp(-1.0 + 1.0e-6, 1.0 - 1.0e-6)
    pre_tanh = torch.atanh(bounded)
    correction = torch.log(1.0 - bounded.square() + 1.0e-6)
    return (self._distribution.log_prob(pre_tanh) - correction).sum(dim=-1)

  def kl_divergence(
    self,
    old_params: tuple[torch.Tensor, ...],
    new_params: tuple[torch.Tensor, ...],
  ) -> torch.Tensor:
    old_mean, old_std = old_params
    new_mean, new_std = new_params
    return torch.distributions.kl_divergence(
      Normal(old_mean, old_std), Normal(new_mean, new_std)
    ).sum(dim=-1)
