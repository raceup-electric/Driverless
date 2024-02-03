class MP(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  m : __torch__.torch.nn.modules.pooling.___torch_mangle_62.MaxPool2d
  def forward(self: __torch__.models.common.___torch_mangle_63.MP,
    argument_1: Tensor) -> Tensor:
    m = self.m
    return (m).forward(argument_1, )
