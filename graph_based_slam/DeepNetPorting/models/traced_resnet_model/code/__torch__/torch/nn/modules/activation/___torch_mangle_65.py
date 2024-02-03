class ReLU(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.torch.nn.modules.activation.___torch_mangle_65.ReLU,
    argument_1: Tensor) -> Tensor:
    return torch.relu_(argument_1)
  def forward1(self: __torch__.torch.nn.modules.activation.___torch_mangle_65.ReLU,
    input: Tensor) -> Tensor:
    return torch.relu_(input)
