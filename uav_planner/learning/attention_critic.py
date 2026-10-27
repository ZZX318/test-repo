"""Small attention critic compatible with per-UAV observations.

PyTorch is intentionally optional: importing this module without torch raises
an actionable error only when a network is instantiated.
"""

from __future__ import annotations


def _require_torch():
    try:
        import torch
        from torch import nn
        return torch, nn
    except ImportError as exc:
        raise RuntimeError("install the optional torch extra to use AttentionCritic") from exc


def build_attention_critic(obs_dim: int, action_dim: int, embed_dim: int = 128,
                           heads: int = 4):
    torch, nn = _require_torch()

    class AttentionCritic(nn.Module):
        def __init__(self):
            super().__init__()
            self.encoder = nn.Sequential(nn.Linear(obs_dim + action_dim, embed_dim), nn.LayerNorm(embed_dim), nn.ReLU())
            self.attention = nn.MultiheadAttention(embed_dim, heads, batch_first=True)
            self.head = nn.Sequential(nn.Linear(embed_dim, embed_dim), nn.ReLU(), nn.Linear(embed_dim, 1))

        def forward(self, observations, actions, return_attention=False):
            tokens = self.encoder(torch.cat([observations, actions], dim=-1))
            attended, weights = self.attention(tokens, tokens, tokens, need_weights=return_attention)
            q = self.head(attended).squeeze(-1)
            return (q, weights) if return_attention else q

    return AttentionCritic()
