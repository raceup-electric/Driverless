class BatchNorm2d(Module):
  __parameters__ = ["weight", "bias", ]
  __buffers__ = ["running_mean", "running_var", "num_batches_tracked", ]
  weight : Tensor
  bias : Tensor
  running_mean : Tensor
  running_var : Tensor
  num_batches_tracked : Tensor
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.torch.nn.modules.batchnorm.___torch_mangle_67.BatchNorm2d,
    argument_1: Tensor) -> Tensor:
    running_var = self.running_var
    running_mean = self.running_mean
    bias = self.bias
    weight = self.weight
    num_batches_tracked = self.num_batches_tracked
    _0 = torch.add_(num_batches_tracked, CONSTANTS.c0)
    out = torch.batch_norm(argument_1, weight, bias, running_mean, running_var, True, 0.10000000000000001, 1.0000000000000001e-05, True)
    return out
