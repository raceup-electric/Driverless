class SiLU(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.utils.activations.___torch_mangle_222.SiLU,
    argument_1: Tensor) -> Tensor:
    _0 = torch.mul(argument_1, torch.sigmoid(argument_1))
    return _0
