class SiLU(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.utils.activations.___torch_mangle_28.SiLU,
    argument_1: Tensor) -> Tensor:
    input = torch.mul(argument_1, torch.sigmoid(argument_1))
    return input
