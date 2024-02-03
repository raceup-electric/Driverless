class SiLU(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.torch.nn.modules.activation.SiLU,
    argument_1: Tensor) -> Tensor:
    return torch.silu_(argument_1)
