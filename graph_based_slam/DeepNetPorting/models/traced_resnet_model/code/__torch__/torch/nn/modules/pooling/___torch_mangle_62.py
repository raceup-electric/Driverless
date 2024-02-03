class MaxPool2d(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.torch.nn.modules.pooling.___torch_mangle_62.MaxPool2d,
    argument_1: Tensor) -> Tensor:
    input = torch.max_pool2d(argument_1, [3, 3], [2, 2], [1, 1], [1, 1])
    return input
